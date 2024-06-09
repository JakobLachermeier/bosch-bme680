use self::i2c_helper::I2CHelper;
use crate::bitfields::RawConfig;
use crate::config::{Configuration, SensorMode, Variant};
use crate::constants::LEN_CONFIG;
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
    sensor_config: RawConfig<[u8; LEN_CONFIG]>,
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
        let sensor_config = self
            .i2c
            .set_config(sensor_config, &calibration_data)
            .await?;
        let variant = self.i2c.get_variant_id().await?;
        self.state = Some(State {
            calibration_data,
            sensor_config,
            variant,
        });
        Ok(())
    }

    /// Returns the wrapped i2c interface
    pub fn into_inner(self) -> I2C {
        self.i2c.into_inner()
    }

    pub async fn set_configuration(&mut self, config: &Configuration) -> Result<(), BmeError<I2C>> {
        let state = self.state.as_mut().ok_or(BmeError::Uninitialized)?;
        self.i2c.set_mode(SensorMode::Sleep).await?;
        let new_config = self.i2c.set_config(config, &state.calibration_data).await?;
        // current conf is used to calculate measurement delay period
        state.sensor_config = new_config;
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
        let delay_period = state.sensor_config.calculate_delay_period_us();
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
    // TODO: embedded_hal_mock doesn't currently have support for the async I2C
    // trait. When that's added, we should add tests here.
}
