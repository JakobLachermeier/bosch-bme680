use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use log::debug;

use crate::bitfields::{CtrlMeasurment, RawConfig, RawData};
use crate::config::{Configuration, GasConfig, SensorMode, Variant};
use crate::constants::{
    ADDRS_CONFIG, ADDR_CONFIG, ADDR_CONTROL_MODE, ADDR_GAS_WAIT_0, ADDR_RES_HEAT_0,
    ADDR_SENSOR_RESULT, ADDR_VARIANT_ID, DELAY_PERIOD_US, LEN_CONFIG,
};
use crate::{
    config::DeviceAddress,
    constants::{
        ADDR_CHIP_ID, ADDR_REG_COEFF1, ADDR_REG_COEFF2, ADDR_REG_COEFF3, ADDR_SOFT_RESET, CHIP_ID,
        CMD_SOFT_RESET, LEN_COEFF1, LEN_COEFF2, LEN_COEFF_ALL,
    },
    data::CalibrationData,
    error::BmeError,
};

// needed to convert h1 and h2 calibration parameters
const BME68X_IDX_H1_MSB: u16 = 25;
const BME68X_IDX_H1_LSB: u16 = 24;
const BME68X_IDX_H2_MSB: u16 = 23;
const BME68X_IDX_H2_LSB: u16 = 24;
const BME68X_BIT_H1_DATA_MSK: u16 = 0x0f;
const BME68X_RHRANGE_MSK: u8 = 0x30;
const BME68X_RSERROR_MSK: u8 = 0xf0;

pub struct I2CHelper<I2C, D> {
    i2c_interface: I2C,
    address: u8,
    delayer: D,
    pub ambient_temperature: i32,
}
impl<I2C, D> I2CHelper<I2C, D>
where
    I2C: I2c<SevenBitAddress>,
    D: DelayNs,
{
    pub fn new(
        i2c_interface: I2C,
        device_address: DeviceAddress,
        delayer: D,
        ambient_temperature: i32,
    ) -> Result<Self, BmeError<I2C>> {
        Self {
            i2c_interface,
            address: device_address.into(),
            delayer,
            // current ambient temperature. Needed to calculate the target temperature of the heater
            ambient_temperature,
        }
        .init()
    }

    pub fn into_inner(self) -> I2C {
        self.i2c_interface
    }
    // pause for duration in us
    pub fn delay(&mut self, duration_us: u32) {
        self.delayer.delay_us(duration_us);
    }
    fn get_register(&mut self, address: u8) -> Result<u8, BmeError<I2C>> {
        debug!("    Getting register: {address:x}.");
        let mut buffer = [0; 1];
        self.i2c_interface
            .write_read(self.address, &[address], &mut buffer)
            .map_err(BmeError::WriteReadError)?;
        Ok(buffer[0])
    }
    pub fn get_registers(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), BmeError<I2C>> {
        debug!(
            "   Getting register: {address:x} to {:x}. Length {} bytes.",
            buffer.len() + address as usize,
            buffer.len()
        );
        self.i2c_interface
            .write_read(self.address, &[address], buffer)
            .map_err(BmeError::WriteReadError)?;
        Ok(())
    }
    fn set_register(&mut self, address: u8, value: u8) -> Result<(), BmeError<I2C>> {
        debug!("    Setting register {address:x} to {value:b}");
        self.i2c_interface
            .write(self.address, &[address, value])
            .map_err(BmeError::WriteError)
    }

    // takes register pairs like [(addr, val), (addr, val)]
    fn set_registers_iter<'a>(
        &mut self,
        register_pairs: impl Iterator<Item = (&'a u8, &'a u8)>,
    ) -> Result<(), BmeError<I2C>> {
        for (address, value) in register_pairs {
            self.set_register(*address, *value)?;
        }
        Ok(())
    }
    /// Soft resets and checks device if device id matches the expected device id
    fn init(mut self) -> Result<Self, BmeError<I2C>> {
        self.soft_reset()?;
        self.delayer.delay_us(DELAY_PERIOD_US);
        let chip_id = self.get_chip_id()?;
        if chip_id == CHIP_ID {
            Ok(self)
        } else {
            Err(BmeError::UnexpectedChipId(chip_id))
        }
    }
    pub fn soft_reset(&mut self) -> Result<(), BmeError<I2C>> {
        debug!("Soft resetting");
        self.set_register(ADDR_SOFT_RESET, CMD_SOFT_RESET)
    }
    fn get_chip_id(&mut self) -> Result<u8, BmeError<I2C>> {
        debug!("Getting chip id");
        self.get_register(ADDR_CHIP_ID)
    }
    pub fn get_variant_id(&mut self) -> Result<Variant, BmeError<I2C>> {
        debug!("Getting variant id");
        Ok(self.get_register(ADDR_VARIANT_ID)?.into())
    }
    // fills buffer with content from 3 seperate reads
    pub fn get_calibration_data(&mut self) -> Result<CalibrationData, BmeError<I2C>> {
        debug!("Getting calibration data");
        let mut coeff_buffer = [0; LEN_COEFF_ALL];
        // fill coeff buffer
        debug!("Filling register buffer 1");
        self.get_registers(ADDR_REG_COEFF1, &mut coeff_buffer[0..LEN_COEFF1])?;
        debug!("Filling register buffer 2");
        self.get_registers(
            ADDR_REG_COEFF2,
            &mut coeff_buffer[LEN_COEFF1..LEN_COEFF1 + LEN_COEFF2],
        )?;
        debug!("Filling register buffer 3");
        self.get_registers(
            ADDR_REG_COEFF3,
            &mut coeff_buffer[LEN_COEFF1 + LEN_COEFF2..LEN_COEFF_ALL],
        )?;
        Ok(extract_calibration_data(coeff_buffer))
    }
    /// Puts the sensor to sleep and adjusts `SensorMode` afterwards
    pub fn set_mode(&mut self, mode: SensorMode) -> Result<(), BmeError<I2C>> {
        // 1. Read ctr_meas register
        // 2. Set last 2 bits to 00 (sleep) if not already in sleep mode
        // 3. Set last 2 bits to 01 (forced) if the requested mode is forced. Do nothing if the requested mode is sleep,
        // as the sensor has already been sent to sleep before.
        debug!("Setting mode to {mode:?}");
        let mut control_register = loop {
            debug!("Getting control register");
            let mut control_register = CtrlMeasurment(self.get_register(ADDR_CONTROL_MODE)?);

            debug!("Current control_register: {control_register:?}");
            let current_mode = control_register.mode();
            debug!("Current mode: {current_mode:?}");
            // Put sensor to sleep unless it already in sleep mode. Same as in the reference implementation
            match current_mode {
                SensorMode::Sleep => break control_register,
                SensorMode::Forced => {
                    control_register.set_mode(SensorMode::Sleep);
                    debug!("Setting control register to: {control_register:?}");
                    self.set_register(ADDR_CONTROL_MODE, control_register.0)?;
                    self.delayer.delay_us(DELAY_PERIOD_US);
                }
            }
        };
        debug!("Broke out of loop with control register: {control_register:?}");
        match mode {
            SensorMode::Sleep => Ok(()),
            SensorMode::Forced => {
                // Change to forced mode. Last two bits=01.
                control_register.set_mode(SensorMode::Forced);
                debug!("Setting control register to: {control_register:?}");
                self.set_register(ADDR_CONTROL_MODE, control_register.0)
            }
        }
    }
    pub fn get_config(&mut self) -> Result<RawConfig<[u8; LEN_CONFIG]>, BmeError<I2C>> {
        debug!("Getting config");
        let mut buffer = [0; LEN_CONFIG];
        self.get_registers(ADDR_CONFIG, &mut buffer)?;
        Ok(RawConfig(buffer))
    }
    /// Gets current config and applies all present values in given config
    /// Returns the new raw config
    pub fn set_config(
        &mut self,
        conf: &Configuration,
        calibration_data: &CalibrationData,
    ) -> Result<RawConfig<[u8; LEN_CONFIG]>, BmeError<I2C>> {
        let mut current_conf = self.get_config()?;
        current_conf.apply_config(conf);

        let pairs = ADDRS_CONFIG.iter().zip(current_conf.0.iter());
        debug!("Setting config registers");
        self.set_registers_iter(pairs)?;
        if let Some(gas_conf) = &conf.gas_config {
            self.set_gas_config(gas_conf, calibration_data)?;
        }
        Ok(current_conf)
    }
    fn set_gas_config(
        &mut self,
        gas_config: &GasConfig,
        calibration_data: &CalibrationData,
    ) -> Result<(), BmeError<I2C>> {
        let gas_wait = gas_config.calc_gas_wait();
        let res_heat = gas_config.calc_res_heat(calibration_data, self.ambient_temperature);
        debug!("Setting gas_wait_0 to {gas_wait}");
        debug!("Setting res_heat_0 to {res_heat}");
        self.set_register(ADDR_GAS_WAIT_0, gas_wait)?;
        self.set_register(ADDR_RES_HEAT_0, res_heat)?;
        Ok(())
    }
    /// Get raw sensor data. 15 bytes starting at 0x1D
    pub fn get_field_data(&mut self) -> Result<RawData<[u8; 15]>, BmeError<I2C>> {
        let mut buffer: [u8; 15] = [0; 15];
        self.get_registers(ADDR_SENSOR_RESULT, &mut buffer)?;
        Ok(RawData(buffer))
    }
}
pub fn extract_calibration_data(coeff_buffer: [u8; 42]) -> CalibrationData {
    let par_t1 = u16::from_be_bytes([coeff_buffer[32], coeff_buffer[31]]);
    let par_t2 = i16::from_be_bytes([coeff_buffer[1], coeff_buffer[0]]);
    let par_t3 = coeff_buffer[2] as i8;

    let par_p1 = u16::from_be_bytes([coeff_buffer[5], coeff_buffer[4]]);
    let par_p2 = i16::from_be_bytes([coeff_buffer[7], coeff_buffer[6]]);
    let par_p3 = coeff_buffer[8] as i8;
    let par_p4 = i16::from_be_bytes([coeff_buffer[11], coeff_buffer[10]]);
    let par_p5 = i16::from_be_bytes([coeff_buffer[13], coeff_buffer[12]]);

    // Switch p6 and p7 to match calibration data from original Bme68x c library
    let par_p6 = coeff_buffer[15] as i8;
    let par_p7 = coeff_buffer[14] as i8;

    let par_p8 = i16::from_be_bytes([coeff_buffer[19], coeff_buffer[18]]);
    let par_p9 = i16::from_be_bytes([coeff_buffer[21], coeff_buffer[20]]);
    let par_p10 = coeff_buffer[22];

    // https://github.com/BoschSensortec/BME68x-Sensor-API/blob/master/bme68x.c#L1807
    let par_h1 = (u16::from(coeff_buffer[BME68X_IDX_H1_MSB as usize]) << 4)
        | (u16::from(coeff_buffer[BME68X_IDX_H1_LSB as usize]) & BME68X_BIT_H1_DATA_MSK);
    // https://github.com/BoschSensortec/BME68x-Sensor-API/blob/master/bme68x.c#L1810
    let par_h2 = (u16::from(coeff_buffer[BME68X_IDX_H2_MSB as usize]) << 4)
        | (u16::from(coeff_buffer[BME68X_IDX_H2_LSB as usize]) >> 4);
    let par_h3 = coeff_buffer[26] as i8;
    let par_h4 = coeff_buffer[27] as i8;
    let par_h5 = coeff_buffer[28] as i8;
    let par_h6 = coeff_buffer[29];
    let par_h7 = coeff_buffer[30] as i8;

    let par_gh1 = coeff_buffer[35] as i8;
    let par_gh2 = i16::from_be_bytes([coeff_buffer[34], coeff_buffer[33]]);
    let par_gh3 = coeff_buffer[36] as i8;

    //let res_heat_range = (coeff_buffer[39] & BME68X_RHRANGE_MSK) / 16;
    let res_heat_range = (coeff_buffer[39] & BME68X_RHRANGE_MSK) / 16;
    let res_heat_val = coeff_buffer[37] as i8;
    let range_sw_err = (coeff_buffer[41] as i8 & BME68X_RSERROR_MSK as i8) / 16;

    CalibrationData {
        par_t1,
        par_t2,
        par_t3,
        par_p1,
        par_p2,
        par_p3,
        par_p4,
        par_p5,
        par_p6,
        par_p7,
        par_p8,
        par_p9,
        par_p10,
        par_h1,
        par_h2,
        par_h3,
        par_h4,
        par_h5,
        par_h6,
        par_h7,
        par_gh1,
        par_gh2,
        par_gh3,
        res_heat_range,
        res_heat_val,
        range_sw_err,
    }
}
#[cfg(test)]
mod i2c_tests {
    extern crate std;
    use super::I2CHelper;
    use crate::{
        config::DeviceAddress,
        constants::{ADDR_CHIP_ID, ADDR_SOFT_RESET, CHIP_ID, CMD_SOFT_RESET},
    };
    use embedded_hal_mock::eh1::{
        delay::NoopDelay,
        i2c::{Mock as I2cMock, Transaction as I2cTransaction},
    };
    use std::vec;
    use std::vec::Vec;
    // primary device address
    const DEVICE_ADDRESS: u8 = 0x76;
    fn setup() -> Vec<I2cTransaction> {
        let transactions = vec![
            // reset chip
            I2cTransaction::write(DEVICE_ADDRESS, vec![ADDR_SOFT_RESET, CMD_SOFT_RESET]),
            // get chip id
            I2cTransaction::write_read(DEVICE_ADDRESS, vec![ADDR_CHIP_ID], vec![CHIP_ID]),
        ];
        transactions
    }
    // i2c mock tests
    #[test]
    fn test_setup_helper() {
        let transactions = setup();
        let i2c_interface = I2cMock::new(&transactions);
        let i2c_helper =
            I2CHelper::new(i2c_interface, DeviceAddress::Primary, NoopDelay {}, 20).unwrap();
        i2c_helper.into_inner().done();
    }
}
