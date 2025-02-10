//! This a pure rust crate to read out sensor data from the [BME680](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/) environmental sensor from bosch.
//!
//! Notes:
//! This library only supports reading out data with I²C but not SPI and
//! only works for the BME680 and NOT for the BME688 though this could be implemented.
//! The [official](https://github.com/BoschSensortec/BME68x-Sensor-API/) c implementation from Bosch was used as a reference.
//!
//! For further information about the sensors capabilities and settings refer to the official [product page](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/).[]
//!
//!
//! ## [`embedded-hal-async`] usage
//!
//! This crate has optional support for [`embedded-hal-async`], which provides
//! asynchronous versions of the [`embedded-hal`] traits. To avoid an unnecessary
//! dependency on `embedded-hal-async` for projects which do not require it, the
//! `embedded-hal-async` support is an optional feature.
//!
//! In order to use the `embedded-hal-async` driver, add the following to your
//! `Cargo.toml`:
//!
//! ```toml
//! [dependencies]
//! bosch-bme680 = { version = "1.0.3", features = ["embedded-hal-async"] }
//! ```
//!
//! Then, construct an instance of the `AsyncBme680` struct using the
//! `embedded_hal_async` `I2c` and `Delay` traits.
//!
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal-async
//! [`embedded-hal-async`]: https://crates.io/crates/embedded-hal-async

// TODO add example here
#![no_std]
#![forbid(unsafe_code)]
#![cfg_attr(docsrs, feature(doc_cfg, doc_auto_cfg))]
#![allow(clippy::excessive_precision)]
#![allow(clippy::unusual_byte_groupings)]

use self::config::{SensorMode, Variant};

use data::CalibrationData;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use i2c_helper::I2CHelper;

pub use self::config::{Configuration, DeviceAddress, GasConfig, IIRFilter, Oversampling};
use crate::data::{calculate_humidity, calculate_pressure, calculate_temperature};
pub use data::MeasurmentData;
pub use error::BmeError;

#[cfg(feature = "embedded-hal-async")]
mod async_impl;
#[cfg(feature = "embedded-hal-async")]
pub use async_impl::AsyncBme680;
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
    current_sensor_config: Configuration,
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
        i2c.set_config(sensor_config, &calibration_data)?;
        let variant = i2c.get_variant_id()?;
        let bme = Self {
            i2c,
            calibration_data,
            current_sensor_config: sensor_config.clone(),
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
        self.i2c.set_config(config, &self.calibration_data)?;
        // current conf is used to calculate measurement delay period
        self.current_sensor_config = config.clone();
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
        let delay_period = self.current_sensor_config.calculate_delay_period_us();
        self.i2c.delay(delay_period);
        // try read new values 5 times and delay if no new data is available or the sensor is still measuring
        for _i in 0..5 {
            let raw_data = self.i2c.get_field_data()?;
            match MeasurmentData::from_raw(raw_data, &self.calibration_data, &self.variant) {
                Some(data) => {
                    // update the current ambient temperature which is needed to calculate the target heater temp
                    self.i2c.ambient_temperature = data.temperature as i32;
                    return Ok(data);
                }
                None => self.i2c.delay(delay_period),
            }
        }
        // Shouldn't happen
        Err(BmeError::MeasuringTimeOut)
    }

    pub fn get_calibration_data(&self) -> &CalibrationData {
        &self.calibration_data
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
    use crate::bitfields::RawConfig;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

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
