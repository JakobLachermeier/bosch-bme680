use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::{I2c, SevenBitAddress};
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
    i2c_helper,
};

pub(crate) struct I2CHelper<I2C, D> {
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
    ) -> Self {
        Self {
            i2c_interface,
            address: device_address.into(),
            delayer,
            // current ambient temperature. Needed to calculate the target temperature of the heater
            ambient_temperature,
        }
    }

    pub fn into_inner(self) -> I2C {
        self.i2c_interface
    }
    // pause for duration in us
    pub async fn delay(&mut self, duration_us: u32) {
        self.delayer.delay_us(duration_us).await;
    }
    async fn get_register(&mut self, address: u8) -> Result<u8, BmeError<I2C>> {
        debug!("    Getting register: {address:x}.");
        let mut buffer = [0; 1];
        self.i2c_interface
            .write_read(self.address, &[address], &mut buffer)
            .await
            .map_err(BmeError::WriteReadError)?;
        Ok(buffer[0])
    }
    pub async fn get_registers(
        &mut self,
        address: u8,
        buffer: &mut [u8],
    ) -> Result<(), BmeError<I2C>> {
        debug!(
            "   Getting register: {address:x} to {:x}. Length {} bytes.",
            buffer.len() + address as usize,
            buffer.len()
        );
        self.i2c_interface
            .write_read(self.address, &[address], buffer)
            .await
            .map_err(BmeError::WriteReadError)?;
        Ok(())
    }
    async fn set_register(&mut self, address: u8, value: u8) -> Result<(), BmeError<I2C>> {
        debug!("    Setting register {address:x} to {value:b}");
        self.i2c_interface
            .write(self.address, &[address, value])
            .await
            .map_err(BmeError::WriteError)
    }

    // takes register pairs like [(addr, val), (addr, val)]
    async fn set_registers_iter<'a>(
        &mut self,
        register_pairs: impl Iterator<Item = (&'a u8, &'a u8)>,
    ) -> Result<(), BmeError<I2C>> {
        for (address, value) in register_pairs {
            self.set_register(*address, *value).await?;
        }
        Ok(())
    }
    /// Soft resets and checks device if device id matches the expected device id
    pub async fn init(&mut self) -> Result<(), BmeError<I2C>> {
        self.soft_reset().await?;
        self.delayer.delay_us(DELAY_PERIOD_US).await;
        let chip_id = self.get_chip_id().await?;
        if chip_id == CHIP_ID {
            Ok(())
        } else {
            Err(BmeError::UnexpectedChipId(chip_id))
        }
    }
    pub async fn soft_reset(&mut self) -> Result<(), BmeError<I2C>> {
        debug!("Soft resetting");
        self.set_register(ADDR_SOFT_RESET, CMD_SOFT_RESET).await
    }
    async fn get_chip_id(&mut self) -> Result<u8, BmeError<I2C>> {
        debug!("Getting chip id");
        self.get_register(ADDR_CHIP_ID).await
    }
    pub async fn get_variant_id(&mut self) -> Result<Variant, BmeError<I2C>> {
        debug!("Getting variant id");
        Ok(self.get_register(ADDR_VARIANT_ID).await?.into())
    }
    // fills buffer with content from 3 seperate reads
    pub async fn get_calibration_data(&mut self) -> Result<CalibrationData, BmeError<I2C>> {
        debug!("Getting calibration data");
        let mut coeff_buffer = [0; LEN_COEFF_ALL];
        // fill coeff buffer
        debug!("Filling register buffer 1");
        self.get_registers(ADDR_REG_COEFF1, &mut coeff_buffer[0..LEN_COEFF1])
            .await?;
        debug!("Filling register buffer 2");
        self.get_registers(
            ADDR_REG_COEFF2,
            &mut coeff_buffer[LEN_COEFF1..LEN_COEFF1 + LEN_COEFF2],
        )
        .await?;
        debug!("Filling register buffer 3");
        self.get_registers(
            ADDR_REG_COEFF3,
            &mut coeff_buffer[LEN_COEFF1 + LEN_COEFF2..LEN_COEFF_ALL],
        )
        .await?;
        Ok(i2c_helper::extract_calibration_data(coeff_buffer))
    }
    /// Puts the sensor to sleep and adjusts `SensorMode` afterwards
    pub async fn set_mode(&mut self, mode: SensorMode) -> Result<(), BmeError<I2C>> {
        // 1. Read ctr_meas register
        // 2. Set last 2 bits to 00 (sleep) if not already in sleep mode
        // 3. Set last 2 bits to 01 (forced) if the requested mode is forced. Do nothing if the requested mode is sleep,
        // as the sensor has already been sent to sleep before.
        debug!("Setting mode to {mode:?}");
        let mut control_register = loop {
            debug!("Getting control register");
            let mut control_register = CtrlMeasurment(self.get_register(ADDR_CONTROL_MODE).await?);

            debug!("Current control_register: {control_register:?}");
            let current_mode = control_register.mode();
            debug!("Current mode: {current_mode:?}");
            // Put sensor to sleep unless it already in sleep mode. Same as in the reference implementation
            match current_mode {
                SensorMode::Sleep => break control_register,
                SensorMode::Forced => {
                    control_register.set_mode(SensorMode::Sleep);
                    debug!("Setting control register to: {control_register:?}");
                    self.set_register(ADDR_CONTROL_MODE, control_register.0)
                        .await?;
                    self.delayer.delay_us(DELAY_PERIOD_US).await;
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
                    .await
            }
        }
    }
    pub async fn get_config(&mut self) -> Result<RawConfig<[u8; LEN_CONFIG]>, BmeError<I2C>> {
        debug!("Getting config");
        let mut buffer = [0; LEN_CONFIG];
        self.get_registers(ADDR_CONFIG, &mut buffer).await?;
        Ok(RawConfig(buffer))
    }
    /// Gets current config and applies all present values in given config
    /// Returns the new raw config
    pub async fn set_config(
        &mut self,
        conf: &Configuration,
        calibration_data: &CalibrationData,
    ) -> Result<(), BmeError<I2C>> {
        let mut current_conf = self.get_config().await?;
        current_conf.apply_config(conf);

        let pairs = ADDRS_CONFIG.iter().zip(current_conf.0.iter());
        debug!("Setting config registers");
        self.set_registers_iter(pairs).await?;
        if let Some(gas_conf) = &conf.gas_config {
            self.set_gas_config(gas_conf, calibration_data).await?;
        }
        Ok(())
    }
    async fn set_gas_config(
        &mut self,
        gas_config: &GasConfig,
        calibration_data: &CalibrationData,
    ) -> Result<(), BmeError<I2C>> {
        let gas_wait = gas_config.calc_gas_wait();
        let res_heat = gas_config.calc_res_heat(calibration_data, self.ambient_temperature);
        debug!("Setting gas_wait_0 to {gas_wait}");
        debug!("Setting res_heat_0 to {res_heat}");
        self.set_register(ADDR_GAS_WAIT_0, gas_wait).await?;
        self.set_register(ADDR_RES_HEAT_0, res_heat).await?;
        Ok(())
    }
    /// Get raw sensor data. 15 bytes starting at 0x1D
    pub async fn get_field_data(&mut self) -> Result<RawData<[u8; 15]>, BmeError<I2C>> {
        let mut buffer: [u8; 15] = [0; 15];
        self.get_registers(ADDR_SENSOR_RESULT, &mut buffer).await?;
        Ok(RawData(buffer))
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
    #[tokio::test]
    async fn test_setup_helper() {
        let transactions = setup();
        let i2c_interface = I2cMock::new(&transactions);
        let mut i2c_helper =
            I2CHelper::new(i2c_interface, DeviceAddress::Primary, NoopDelay {}, 20);
        i2c_helper.init().await.unwrap();
        i2c_helper.into_inner().done();
    }
}
