//! This a pure rust crate to read out sensor data from the [BME680](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/) environmental sensor from bosch.
//!
//! Notes:
//! This library only supports reading out data with I²C but not SPI and
//! only works for the BME680 and NOT for the BME688 though this could be implemented.
//! The [official](https://github.com/BoschSensortec/BME68x-Sensor-API/) c implementation from Bosch was used as a reference.
//!
//! For further information about the sensors capabilities and settings refer to the official [product page](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/).

// TODO add example here
#![no_std]
#![forbid(unsafe_code)]

use self::config::{SensorMode, Variant};
use bitfields::RawConfig;
use constants::{
    CYCLE_DURATION, GAS_MEAS_DURATION, LEN_CONFIG, TPH_SWITCHING_DURATION, WAKEUP_DURATION,
};
use data::CalibrationData;
use embedded_hal::{
    delay::DelayNs,
};
use embedded_hal::i2c::{I2c, SevenBitAddress};
use i2c_helper::I2CHelper;

pub use self::config::{Configuration, DeviceAddress, GasConfig, IIRFilter, Oversampling};
use crate::data::{calculate_humidity, calculate_pressure, calculate_temperature};
pub use data::MeasurmentData;
pub use error::BmeError;

mod bitfields;
mod calculations;
mod config;
mod constants;
mod data;
mod error;
mod i2c_helper;

/// Sensor driver
pub struct Bme680<I2C, D> {
    // actually communicates with sensor
    i2c: I2CHelper<I2C, D>,
    // calibration data that was saved on the sensor
    calibration_data: CalibrationData,
    // used to calculate measurement delay period
    sensor_config: RawConfig<[u8; LEN_CONFIG]>,
    // needed to calculate the gas resistance since it differs between bme680 and bme688
    variant: Variant,
}
impl<I2C, D> Bme680<I2C, D>
where
    I2C: I2c<SevenBitAddress>,
    // <I2C as WriteRead>::Error: core::fmt::Debug,
    // <I2C as Write>::Error: core::fmt::Debug,
    D: DelayNs,
{
    /// Creates a new instance of the Sensor
    ///
    /// # Arguments
    /// * `delayer` - Used to wait for the triggered measurement to finish
    /// * `ambient_temperature` - Needed to calculate the heater target temperature
    pub fn new(
        i2c_interface: I2C,
        device_address: DeviceAddress,
        delayer: D,
        sensor_config: &Configuration,
        ambient_temperature: i32,
    ) -> Result<Self, BmeError<I2C>> {
        let mut i2c = I2CHelper::new(i2c_interface, device_address, delayer, ambient_temperature)?;

        let calibration_data = i2c.get_calibration_data()?;
        let sensor_config = i2c.set_config(sensor_config, &calibration_data)?;
        let variant = i2c.get_variant_id()?;
        let bme = Self {
            i2c,
            calibration_data,
            sensor_config,
            variant,
        };

        Ok(bme)
    }
    /// Returns the wrapped i2c interface
    pub fn into_inner(self) -> I2C {
        self.i2c.into_inner()
    }

    fn put_to_sleep(&mut self) -> Result<(), BmeError<I2C>> {
        self.i2c.set_mode(SensorMode::Sleep)
    }
    pub fn set_configuration(&mut self, config: &Configuration) -> Result<(), BmeError<I2C>> {
        self.put_to_sleep()?;
        let new_config = self.i2c.set_config(config, &self.calibration_data)?;
        // current conf is used to calculate measurement delay period
        self.sensor_config = new_config;
        Ok(())
    }
    /// Trigger a new measurement.
    /// # Errors
    /// If no new data is generated in 5 tries a Timeout error is returned.
    // Sets the sensor mode to forced
    // Tries to wait 5 times for new data with a delay calculated based on the set sensor config
    // If no new data could be read in those 5 attempts a Timeout error is returned
    pub fn measure(&mut self) -> Result<MeasurmentData, BmeError<I2C>> {
        self.i2c.set_mode(SensorMode::Forced)?;
        let delay_period = self.calculate_delay_period_us();
        self.i2c.delay(delay_period);
        // try read new values 5 times and delay if no new data is available or the sensor is still measuring
        for _i in 0..5 {
            let raw_data = self.i2c.get_field_data()?;
            if !raw_data.measuring() && raw_data.new_data() {
                let (temperature, t_fine) =
                    calculate_temperature(raw_data.temperature_adc().0, &self.calibration_data);
                // update the current ambient temperature which is needed to calculate the target heater temp
                self.i2c.ambient_temperature = temperature as i32;
                let pressure =
                    calculate_pressure(raw_data.pressure_adc().0, &self.calibration_data, t_fine);
                let humidity =
                    calculate_humidity(raw_data.humidity_adc().0, &self.calibration_data, t_fine);
                let gas_resistance = if raw_data.gas_valid() && !raw_data.gas_measuring() {
                    let gas_resistance = self.variant.calc_gas_resistance(
                        raw_data.gas_adc().0,
                        self.calibration_data.range_sw_err,
                        raw_data.gas_range() as usize,
                    );
                    Some(gas_resistance)
                } else {
                    None
                };

                let data = MeasurmentData {
                    temperature,
                    gas_resistance,
                    humidity,
                    pressure,
                };
                return Ok(data);
            } else {
                self.i2c.delay(delay_period);
            }
        }
        // Shouldn't happen
        Err(BmeError::MeasuringTimeOut)
    }
    // calculates the delay period needed for a measurement in microseconds.
    fn calculate_delay_period_us(&self) -> u32 {
        let mut measurement_cycles: u32 = 0;
        measurement_cycles += self.sensor_config.temperature_oversampling().cycles();
        measurement_cycles += self.sensor_config.humidity_oversampling().cycles();
        measurement_cycles += self.sensor_config.pressure_oversampling().cycles();

        let mut measurement_duration = measurement_cycles * CYCLE_DURATION;
        measurement_duration += TPH_SWITCHING_DURATION;
        measurement_duration += GAS_MEAS_DURATION;

        measurement_duration += WAKEUP_DURATION;

        measurement_duration
    }
}

#[cfg(test)]
mod library_tests {
    extern crate std;

    use std::vec;
    use std::vec::Vec;

    use crate::constants::{
        ADDR_CHIP_ID, ADDR_CONFIG, ADDR_CONTROL_MODE, ADDR_GAS_WAIT_0, ADDR_REG_COEFF1,
        ADDR_REG_COEFF2, ADDR_REG_COEFF3, ADDR_RES_HEAT_0, ADDR_SOFT_RESET, ADDR_VARIANT_ID,
        CHIP_ID, CMD_SOFT_RESET, LEN_COEFF1, LEN_COEFF2, LEN_COEFF3,
    };
    use crate::i2c_helper::extract_calibration_data;

    const CALIBRATION_DATA: [u8; 42] = [
        179, 193, 176, 188, 21, 51, 11, 29, 222, 179, 184, 1, 230, 47, 209, 22, 154, 34, 237, 70,
        148, 134, 44, 13, 204, 61, 206, 69, 18, 43, 124, 164, 92, 132, 19, 63, 29, 28, 201, 140,
        70, 24,
    ];

    use super::*;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    use test_log::test;

    fn setup_transactions() -> Vec<I2cTransaction> {
        let mut transactions = vec![];
        let calibration_data_1 = CALIBRATION_DATA[0..LEN_COEFF1].to_vec();
        let calibration_data_2 = CALIBRATION_DATA[LEN_COEFF1..LEN_COEFF1 + LEN_COEFF2].to_vec();
        let calibration_data_3 = CALIBRATION_DATA
            [LEN_COEFF1 + LEN_COEFF2..LEN_COEFF1 + LEN_COEFF2 + LEN_COEFF3]
            .to_vec();
        assert_eq!(calibration_data_1.len(), LEN_COEFF1);
        assert_eq!(calibration_data_2.len(), LEN_COEFF2);
        assert_eq!(calibration_data_3.len(), LEN_COEFF3);
        // soft reset
        transactions.push(I2cTransaction::write(
            DeviceAddress::Primary.into(),
            vec![ADDR_SOFT_RESET, CMD_SOFT_RESET],
        ));
        // check device id
        transactions.push(I2cTransaction::write_read(
            DeviceAddress::Primary.into(),
            vec![ADDR_CHIP_ID],
            vec![CHIP_ID],
        ));
        // calibration_data
        transactions.push(I2cTransaction::write_read(
            DeviceAddress::Primary.into(),
            vec![ADDR_REG_COEFF1],
            calibration_data_1,
        ));
        transactions.push(I2cTransaction::write_read(
            DeviceAddress::Primary.into(),
            vec![ADDR_REG_COEFF2],
            calibration_data_2,
        ));
        transactions.push(I2cTransaction::write_read(
            DeviceAddress::Primary.into(),
            vec![ADDR_REG_COEFF3],
            calibration_data_3,
        ));
        // set config
        // 1. get current config by writing to register 0x71 with buffer len 5.
        // 2. apply default user facing config to the default values of the sensor.
        // 3. write gas_wait_0 and res_heat_0
        // config defaults to 0s
        // 1.
        let default_config = [0u8; 5];
        transactions.push(I2cTransaction::write_read(
            DeviceAddress::Primary.into(),
            vec![ADDR_CONFIG],
            default_config.into(),
        ));
        // 2.
        let user_config = Configuration::default();
        let mut raw_config = RawConfig(default_config);
        raw_config.apply_config(&user_config);
        // add unique write for each register
        raw_config
            .0
            .into_iter()
            .enumerate()
            .for_each(|(register_offset, register_content)| {
                transactions.push(I2cTransaction::write(
                    DeviceAddress::Primary.into(),
                    vec![ADDR_CONFIG + register_offset as u8, register_content],
                ));
            });
        // 3.
        let gas_config = user_config.gas_config.unwrap();
        let gas_wait_0 = gas_config.calc_gas_wait();
        let res_heat_0 = gas_config.calc_res_heat(&extract_calibration_data(CALIBRATION_DATA), 20);
        transactions.push(I2cTransaction::write(
            DeviceAddress::Primary.into(),
            vec![ADDR_GAS_WAIT_0, gas_wait_0],
        ));
        transactions.push(I2cTransaction::write(
            DeviceAddress::Primary.into(),
            vec![ADDR_RES_HEAT_0, res_heat_0],
        ));
        // get chip variant
        transactions.push(I2cTransaction::write_read(
            DeviceAddress::Primary.into(),
            vec![ADDR_VARIANT_ID],
            vec![0],
        ));
        transactions
    }
    fn add_sleep_to_sleep_transactions(transactions: &mut Vec<I2cTransaction>) {
        transactions.push(I2cTransaction::write_read(
            DeviceAddress::Primary.into(),
            vec![ADDR_CONTROL_MODE],
            // sleep mode
            vec![0b101011_00],
        ));
    }
    #[test]
    fn test_setup() {
        let transactions = setup_transactions();
        let i2c_interface = I2cMock::new(&transactions);
        let bme = Bme680::new(
            i2c_interface,
            DeviceAddress::Primary,
            NoopDelay::new(),
            &Configuration::default(),
            20,
        )
        .unwrap();
        bme.into_inner().done();
    }

    #[test]
    fn test_set_mode_forced_to_sleep() {
        let mut transactions = setup_transactions();
        transactions.push(I2cTransaction::write_read(
            DeviceAddress::Primary.into(),
            vec![ADDR_CONTROL_MODE],
            // sleep mode
            vec![0b101011_01],
        ));
        transactions.push(I2cTransaction::write(
            DeviceAddress::Primary.into(),
            vec![ADDR_CONTROL_MODE, 0b101011_00],
        ));
        transactions.push(I2cTransaction::write_read(
            DeviceAddress::Primary.into(),
            vec![ADDR_CONTROL_MODE],
            // sleep mode
            vec![0b101011_00],
        ));
        // Transactions: Get(Forced) -> Set(Sleep) -> Get(Sleep)
        let i2c_interface = I2cMock::new(&transactions);
        let mut bme = Bme680::new(
            i2c_interface,
            DeviceAddress::Primary,
            NoopDelay::new(),
            &Configuration::default(),
            20,
        )
        .unwrap();
        bme.put_to_sleep().unwrap();
        bme.into_inner().done();
    }
    #[test]
    fn test_set_mode_sleep_to_sleep() {
        let mut transactions = setup_transactions();
        add_sleep_to_sleep_transactions(&mut transactions);
        let i2c_interface = I2cMock::new(&transactions);
        let mut bme = Bme680::new(
            i2c_interface,
            DeviceAddress::Primary,
            NoopDelay::new(),
            &Configuration::default(),
            20,
        )
        .unwrap();
        bme.put_to_sleep().unwrap();
        bme.into_inner().done();
    }
}
