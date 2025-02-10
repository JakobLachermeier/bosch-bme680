use self::i2c_helper::I2CHelper;
use crate::config::{Configuration, SensorMode, Variant};
use crate::data::CalibrationData;
use crate::BmeError;
use crate::DeviceAddress;
use crate::MeasurmentData;
use embedded_hal_async::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};
mod i2c_helper;

/// Asynchronous BME680 sensor driver.
///
/// This struct is similar to the [`Bme680`](crate::Bme680) type, except that it
/// uses the [`embedded_hal_async`] crate's `I2c` and `Delay` traits, rather
/// than the [`embedded_hal`] versions of those traits.
///
/// # Notes
///
/// The [`AsyncBme680::new`] constructor is not asynchronous, and therefore ---
/// unlike the  synchronous [`Bme680::new`] --- it does not initialize the
/// sensor.  Instead, the sensor must be initialized using the
/// [`AsyncBme680::initialize`] method before reading sensor data. Otherwise,
/// the [`AsyncBme680::measure`] method will return [`BmeError::Uninitialized`].
///
/// [`Bme680::new`]: crate::Bme680::new
pub struct AsyncBme680<I2C, D> {
    // actually communicates with sensor
    i2c: I2CHelper<I2C, D>,
    state: Option<State>,
}

struct State {
    // calibration data that was saved on the sensor
    calibration_data: CalibrationData,
    // used to calculate measurement delay period
    current_sensor_config: Configuration,
    // needed to calculate the gas resistance since it differs between bme680 and bme688
    variant: Variant,
}

impl<I2C, D> AsyncBme680<I2C, D>
where
    I2C: I2c<SevenBitAddress>,
    D: DelayNs,
{
    /// Creates a new instance of the Sensor
    ///
    /// # Arguments
    /// * `delayer` - Used to wait for the triggered measurement to finish
    /// * `ambient_temperature` - Needed to calculate the heater target
    ///   temperature
    ///
    /// # Notes
    ///
    /// This constructor is not asynchronous, and therefore --- unlike the
    /// synchronous [`Bme680::new`] --- it does not initialize the sensor.
    /// Instead, the sensor must be initialized using the
    /// [`AsyncBme680::initialize`] method before reading sensor data.
    /// Otherwise, the [`AsyncBme680::measure`] method  will return
    /// [`BmeError::Uninitialized`].
    ///
    /// [`Bme680::new`]: crate::Bme680::new
    pub fn new(
        i2c_interface: I2C,
        device_address: DeviceAddress,
        delayer: D,
        ambient_temperature: i32,
    ) -> Self {
        let i2c = I2CHelper::new(i2c_interface, device_address, delayer, ambient_temperature);

        Self { i2c, state: None }
    }

    pub async fn initialize(&mut self, sensor_config: &Configuration) -> Result<(), BmeError<I2C>> {
        self.i2c.init().await?;
        let calibration_data = self.i2c.get_calibration_data().await?;
        self.i2c
            .set_config(sensor_config, &calibration_data)
            .await?;
        let variant = self.i2c.get_variant_id().await?;
        self.state = Some(State {
            calibration_data,
            current_sensor_config: sensor_config.clone(),
            variant,
        });
        Ok(())
    }
    pub async fn put_to_sleep(&mut self) -> Result<(), BmeError<I2C>> {
        self.i2c.set_mode(SensorMode::Sleep).await
    }
    /// Returns the wrapped i2c interface
    pub fn into_inner(self) -> I2C {
        self.i2c.into_inner()
    }

    pub async fn set_configuration(&mut self, config: &Configuration) -> Result<(), BmeError<I2C>> {
        let state = self.state.as_mut().ok_or(BmeError::Uninitialized)?;
        self.i2c.set_mode(SensorMode::Sleep).await?;
        self.i2c.set_config(config, &state.calibration_data).await?;
        // current conf is used to calculate measurement delay period
        state.current_sensor_config = config.clone();
        Ok(())
    }
    /// Trigger a new measurement.
    /// # Errors
    /// If no new data is generated in 5 tries a Timeout error is returned.
    // Sets the sensor mode to forced
    // Tries to wait 5 times for new data with a delay calculated based on the set sensor config
    // If no new data could be read in those 5 attempts a Timeout error is returned
    pub async fn measure(&mut self) -> Result<MeasurmentData, BmeError<I2C>> {
        let state = self.state.as_mut().ok_or(BmeError::Uninitialized)?;
        self.i2c.set_mode(SensorMode::Forced).await?;
        let delay_period = state.current_sensor_config.calculate_delay_period_us();

        self.i2c.delay(delay_period).await;
        // try read new values 5 times and delay if no new data is available or the sensor is still measuring
        for _i in 0..5 {
            let raw_data = self.i2c.get_field_data().await?;
            match MeasurmentData::from_raw(raw_data, &state.calibration_data, &state.variant) {
                Some(data) => {
                    // update the current ambient temperature which is needed to calculate the target heater temp
                    self.i2c.ambient_temperature = data.temperature as i32;
                    return Ok(data);
                }
                None => self.i2c.delay(delay_period).await,
            }
        }
        // Shouldn't happen
        Err(BmeError::MeasuringTimeOut)
    }

    pub fn get_calibration_data(&self) -> Result<&CalibrationData, BmeError<I2C>> {
        Ok(&self
            .state
            .as_ref()
            .ok_or(BmeError::Uninitialized)?
            .calibration_data)
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
    #[tokio::test]
    async fn test_setup() {
        let transactions = setup_transactions();
        let i2c_interface = I2cMock::new(&transactions);
        let mut bme = AsyncBme680::new(i2c_interface, DeviceAddress::Primary, NoopDelay::new(), 20);
        bme.initialize(&Configuration::default()).await.unwrap();
        bme.into_inner().done();
    }

    #[tokio::test]
    async fn test_set_mode_forced_to_sleep() {
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
        let mut bme = AsyncBme680::new(i2c_interface, DeviceAddress::Primary, NoopDelay::new(), 20);
        bme.initialize(&Configuration::default()).await.unwrap();
        bme.put_to_sleep().await.unwrap();
        bme.into_inner().done();
    }
    #[tokio::test]
    async fn test_set_mode_sleep_to_sleep() {
        let mut transactions = setup_transactions();
        add_sleep_to_sleep_transactions(&mut transactions);
        let i2c_interface = I2cMock::new(&transactions);
        let mut bme = AsyncBme680::new(i2c_interface, DeviceAddress::Primary, NoopDelay::new(), 20);
        bme.initialize(&Configuration::default()).await.unwrap();
        bme.put_to_sleep().await.unwrap();
        bme.into_inner().done();
    }
}
