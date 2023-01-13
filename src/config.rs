use core::time::Duration;

use log::warn;

use crate::{
    constants::{GAS_ARRAY_1, GAS_ARRAY_2, MAX_HEATER_TEMPERATURE, MAX_HEATER_WAIT_DURATION_MS},
    data::CalibrationData,
};

/// Use Primary if SDO connector of the sensor is connected to ground and Secondary if SDO is connected to Vin.
#[repr(u8)]
pub enum DeviceAddress {
    Primary = 0x76,
    Secondary = 0x77,
}

impl From<DeviceAddress> for u8 {
    fn from(value: DeviceAddress) -> Self {
        match value {
            DeviceAddress::Primary => 0x76,
            DeviceAddress::Secondary => 0x77,
        }
    }
}

impl Default for DeviceAddress {
    fn default() -> Self {
        Self::Primary
    }
}
// Variant_id
// gas_low = 0
// gas_high = 1
pub enum Variant {
    GasLow = 0,
    GasHigh = 1,
}
impl From<u8> for Variant {
    fn from(value: u8) -> Self {
        match value {
            0 => Variant::GasLow,
            1 => Variant::GasHigh,
            x => panic!(
                "Got unimplemented device variant from sensor: {x}. Possible values are: [0, 1]."
            ),
        }
    }
}

impl Variant {
    pub fn calc_gas_resistance(
        &self,
        adc_gas: u16,
        range_switching_error: i8,
        gas_range: usize,
    ) -> f32 {
        match self {
            Self::GasLow => {
                let adc_gas = adc_gas as f32;
                let gas_range_f = (1 << gas_range) as f32;
                let var1 = 1340. + (5. * range_switching_error as f32);
                let var2 = var1 * (1. + GAS_ARRAY_1[gas_range] / 100.);
                let var3 = 1. + (GAS_ARRAY_2[gas_range] / 100.);

                1. / (var3 * (0.000000125) * gas_range_f * (((adc_gas - 512.) / var2) + 1.))
            }
            Self::GasHigh => {
                let var1 = 262144_u32 >> gas_range;
                let mut var2 = adc_gas as i32 - 512_i32;
                var2 *= 3;
                var2 += 4096;
                1000000. * var1 as f32 / var2 as f32
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum SensorMode {
    Sleep,
    Forced,
}

impl From<SensorMode> for u8 {
    fn from(value: SensorMode) -> Self {
        match value {
            SensorMode::Sleep => 0,
            SensorMode::Forced => 1,
        }
    }
}
impl From<u8> for SensorMode {
    fn from(val: u8) -> Self {
        match val {
            0 => SensorMode::Sleep,
            1 => SensorMode::Forced,
            invalid => panic!("Failed to read sensor mode. Received {invalid:b} possible values are 0b00(sleep) or 0b01(forced)"),
        }
    }
}

/// Used to enable gas measurment.
/// Default values are 150ms heater duration and 300°C heater target temperature
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GasConfig {
    heater_duration: Duration,
    heater_target_temperature: u16,
    // idac heat is not implemented since the control loop will find the current after a few iterations anyway.
}
impl Default for GasConfig {
    /// Defaults to 150ms heater duration and 300°C heater target temperature
    fn default() -> Self {
        Self {
            heater_duration: Duration::from_millis(150),
            heater_target_temperature: 300,
        }
    }
}
impl GasConfig {
    pub fn calc_gas_wait(&self) -> u8 {
        let mut duration = self.heater_duration.as_millis() as u16;
        let mut factor: u8 = 0;

        if duration >= MAX_HEATER_WAIT_DURATION_MS {
            warn!("Specified heater duration longer than {MAX_HEATER_WAIT_DURATION_MS}ms. Setting to {MAX_HEATER_WAIT_DURATION_MS}ms instead.");
            0xff /* Max duration*/
        } else {
            while duration > 0x3F {
                duration /= 4;
                factor += 1;
            }
            duration as u8 + factor * 64
        }
    }
    pub fn calc_res_heat(
        &self,
        calibration_data: &CalibrationData,
        ambient_temperature: i32,
    ) -> u8 {
        // cap at 400°C
        let target_temperature = if self.heater_target_temperature > MAX_HEATER_TEMPERATURE {
            warn!(
                "Specified heater target temperature higher than {MAX_HEATER_TEMPERATURE}°C. Setting to 400°C instead."  
          );
            400u16
        } else {
            self.heater_target_temperature
        };
        let var1 = ((ambient_temperature * calibration_data.par_gh3 as i32) / 1000) * 256;
        let var2 = (calibration_data.par_gh1 as i32 + 784)
            * (((((calibration_data.par_gh2 as i32 + 154009) * target_temperature as i32 * 5)
                / 100)
                + 3276800)
                / 10);
        let var3 = var1 + (var2 / 2);
        let var4 = var3 / (calibration_data.res_heat_range as i32 + 4);
        let var5 = (131 * calibration_data.res_heat_val as i32) + 65536;
        let heatr_res_x100 = ((var4 / var5) - 250) * 34;
        ((heatr_res_x100 + 50) / 100) as u8
    }
}

/// Used to set Sensor settings.
/// All options not set by the builder are set to default values.
/// 
/// ```rust
/// # use bosch_bme680::{Configuration, Oversampling, IIRFilter};
/// # fn main() {
/// let configuration = Configuration::builder()
///                     .temperature_oversampling(Oversampling::By2)
///                     .pressure_oversampling(Oversampling::By16)
///                     .humidity_oversampling(Oversampling::By1)
///                     .filter(IIRFilter::Coeff1)
///                     // Gas measurment is enabled by default. To disable it pass None as the GasConfig
///                     .gas_config(None)
///                     .build();
///                         
/// # }
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Configuration {
    pub temperature_oversampling: Option<Oversampling>,
    pub pressure_oversampling: Option<Oversampling>,
    pub humidity_oversampling: Option<Oversampling>,
    pub filter: Option<IIRFilter>,
    pub gas_config: Option<GasConfig>,
}

impl Default for Configuration {
    /// Sets sensible default values for all options.
    /// Temperature oversampling: By2, 
    /// Pressure oversampling: By16, 
    /// Humidity oversampling: By1, 
    /// IIRFilter: Coeff1, 
    /// Gas config:
    /// heating duration: 150ms, 
    /// heater target temperature: 300°C
    fn default() -> Self {
        Self {
            temperature_oversampling: Some(Oversampling::By2),
            pressure_oversampling: Some(Oversampling::By16),
            humidity_oversampling: Some(Oversampling::By1),
            filter: Some(IIRFilter::Coeff1),
            gas_config: Some(GasConfig::default()),
        }
    }
}
impl Configuration {
    pub fn builder() -> ConfigBuilder {
        ConfigBuilder {
            config: Configuration::default(),
        }
    }
}
pub struct ConfigBuilder {
    config: Configuration,
}
impl ConfigBuilder {
    pub fn temperature_oversampling(mut self, oversampling: Oversampling) -> Self {
        self.config.temperature_oversampling = Some(oversampling);
        self
    }
    pub fn humidity_oversampling(mut self, oversampling: Oversampling) -> Self {
        self.config.humidity_oversampling = Some(oversampling);
        self
    }
    pub fn pressure_oversampling(mut self, oversampling: Oversampling) -> Self {
        self.config.pressure_oversampling = Some(oversampling);
        self
    }
    pub fn filter(mut self, filter: IIRFilter) -> Self {
        self.config.filter = Some(filter);
        self
    }
    pub fn gas_config(mut self, gas_config: Option<GasConfig>) -> Self {
        self.config.gas_config = gas_config;
        self
    }
    pub fn build(self) -> Configuration {
        self.config
    }
}
/// Oversampling settings for temperature, humidity, pressure
#[derive(Debug, Eq, PartialEq, Clone)]
pub enum Oversampling {
    Skipped,
    By1,
    By2,
    By4,
    By8,
    By16,
}
impl Oversampling {
    pub fn cycles(&self) -> u32 {
        match self {
            Self::Skipped => 0,
            Self::By1 => 1,
            Self::By2 => 2,
            Self::By4 => 4,
            Self::By8 => 8,
            Self::By16 => 16,
        }
    }
}
impl From<u8> for Oversampling {
    fn from(val: u8) -> Self {
        match val {
            0 => Oversampling::Skipped,
            1 => Oversampling::By1,
            2 => Oversampling::By2,
            3 => Oversampling::By4,
            4 => Oversampling::By8,
            _ => Oversampling::By16,
        }
    }
}

impl From<Oversampling> for u8 {
    fn from(value: Oversampling) -> Self {
        match value {
            Oversampling::Skipped => 0,
            Oversampling::By1 => 1,
            Oversampling::By2 => 2,
            Oversampling::By4 => 3,
            Oversampling::By8 => 4,
            Oversampling::By16 => 5,
        }
    }
}

/// IIR filter control applies to temperature and pressure data.
#[derive(Debug, Eq, PartialEq, Clone)]
pub enum IIRFilter {
    Coeff0,
    Coeff1,
    Coeff3,
    Coeff7,
    Coeff15,
    Coeff31,
    Coeff63,
    Coeff127,
}
impl From<u8> for IIRFilter {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Coeff0,
            1 => Self::Coeff1,
            2 => Self::Coeff3,
            3 => Self::Coeff7,
            4 => Self::Coeff15,
            5 => Self::Coeff31,
            6 => Self::Coeff63,
            _ => Self::Coeff127,
        }
    }
}

impl From<IIRFilter> for u8 {
    fn from(value: IIRFilter) -> Self {
        match value {
            IIRFilter::Coeff0 => 0,
            IIRFilter::Coeff1 => 1,
            IIRFilter::Coeff3 => 2,
            IIRFilter::Coeff7 => 3,
            IIRFilter::Coeff15 => 4,
            IIRFilter::Coeff31 => 5,
            IIRFilter::Coeff63 => 6,
            IIRFilter::Coeff127 => 7,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Clone)]
pub enum HeaterProfile {
    Profile0,
    Profile1,
    Profile2,
    Profile3,
    Profile4,
    Profile5,
    Profile6,
    Profile7,
    Profile8,
    Profile9,
}
impl From<u8> for HeaterProfile {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Profile0,
            1 => Self::Profile1,
            2 => Self::Profile2,
            3 => Self::Profile3,
            4 => Self::Profile4,
            5 => Self::Profile5,
            6 => Self::Profile6,
            7 => Self::Profile7,
            8 => Self::Profile8,
            _ => Self::Profile9,
        }
    }
}

impl From<HeaterProfile> for u8 {
    fn from(value: HeaterProfile) -> Self {
        match value {
            HeaterProfile::Profile0 => 0,
            HeaterProfile::Profile1 => 1,
            HeaterProfile::Profile2 => 2,
            HeaterProfile::Profile3 => 3,
            HeaterProfile::Profile4 => 4,
            HeaterProfile::Profile5 => 5,
            HeaterProfile::Profile6 => 6,
            HeaterProfile::Profile7 => 7,
            HeaterProfile::Profile8 => 8,
            HeaterProfile::Profile9 => 9,
        }
    }
}

#[cfg(test)]
mod config_tests {
    extern crate std;
    use std::time::Duration;

    use crate::config::SensorMode;

    use super::GasConfig;

    #[test]
    fn test_sensor_mode() {
        let sleeping = 0u8;
        let forced = 1u8;
        assert!(SensorMode::Sleep == sleeping.into());
        assert!(SensorMode::Forced == forced.into());
    }
    #[test]
    fn test_gas_config() {
        let config = GasConfig {
            heater_duration: Duration::from_millis(100),
            heater_target_temperature: 200,
        };
        assert!(config.calc_gas_wait() <= config.heater_duration.as_millis() as u8);
        // taken from data sheet
        assert!(config.calc_gas_wait() == 0x59);
    }
}
