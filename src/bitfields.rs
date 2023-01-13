use crate::config::{Configuration, HeaterProfile, IIRFilter, Oversampling, SensorMode};
use bitfield::bitfield;
use core::time::Duration;

bitfield! {
    pub struct RawConfig([u8]);
    impl Debug;
    u8;
    // 0x75<4:2>
    pub from into IIRFilter, filter, set_filter : calc_position(4, 4), calc_position(2, 4);
    // 0x74<1:0>
    pub mode, set_mode: calc_position(1, 3), calc_position(0, 3);
    // 0x74<4:2>
    pub from into Oversampling, pressure_oversampling, set_pressure_oversampling : calc_position(4, 3), calc_position(2, 3);
    // 0x74<7:5>
    pub from into Oversampling, temperature_oversampling, set_temperature_oversampling: calc_position(7, 3), calc_position(5, 3);
    // 0x72<2:0>
    pub from into Oversampling, humidity_oversampling, set_humidity_oversampling: calc_position(2, 1), calc_position(0, 1);
    // 0x71<3:0>
    pub from into HeaterProfile, heater_profile, set_heater_profile: calc_position(3, 0), calc_position(0, 0);
    // 0x71<4>
    pub run_gas, set_run_gas: calc_position(4, 0);
}

impl RawConfig<[u8; 5]> {
    /// Applies all present settings in the config.
    /// None values will be ignored and left as they were before.
    /// That means leaving in the default values before configuring and leaving prior set values as they were are.
    /// Does not check for nonsensical configuration settings, such as trying to read the gas meas without specifying a gas config
    pub fn apply_config(&mut self, config: &Configuration) {
        // maybe consume config here
        let config = config.clone();
        if let Some(temperature_oversampling) = config.temperature_oversampling {
            self.set_temperature_oversampling(temperature_oversampling);
        }
        if let Some(pressure_oversampling) = config.pressure_oversampling {
            self.set_pressure_oversampling(pressure_oversampling);
        }
        if let Some(humidity_oversampling) = config.humidity_oversampling {
            self.set_humidity_oversampling(humidity_oversampling);
        }
        if let Some(filter) = config.filter {
            self.set_filter(filter);
        }
        if let Some(_gas_config) = config.gas_config {
            self.set_run_gas(true);
            // Only heater profile0 is needed for forced mode.
            // Sequential mode is not implemented and only available in bme688
            self.set_heater_profile(HeaterProfile::Profile0);
        }
    }
}

bitfield! {
    pub struct RawGasConfig([u8]);
    impl Debug;
    u8;
    pub res_heat, _: 7, 0;
    pub from into GasWaitDuration, gas_wait, _: calc_position(7, 1), calc_position(0, 1);
}
#[derive(Debug)]
pub struct GasWaitDuration(Duration);
impl From<u8> for GasWaitDuration {
    fn from(val: u8) -> Self {
        Self(Duration::from_millis(val as u64))
    }
}

impl From<GasWaitDuration> for Duration {
    fn from(value: GasWaitDuration) -> Self {
        value.0
    }
}

// Ctrl_meas content. Register 0x74
bitfield! {
    pub struct CtrlMeasurment(u8);
    impl Debug;
    u8;
    pub from into Oversampling, temperature_os, set_temperature_os: 7, 5;
    pub from into Oversampling, pressure_os, set_pressure_os: 4, 2;
    pub from into SensorMode, mode, set_mode: 1, 0;
}

bitfield! {
    pub struct MeasurmentStatus(u8);
    impl Debug;
    u8;
    pub bool, new_data, _: 7;
    pub bool, gas_measuring, _: 6;
    pub bool, measuring, _: 5;
    pub gas_meas_index, _: 3, 0;
}

// 15 long
// 0: meas_status_0 0x1D
// 1: _             0x1E
// 2: press_msb     0x2F
// 3: press_lsb     0x20
// 4: press_xlsb    0x21
// 5: temp_msb      0x22
// 6: temp_lsb      0x23
// 7: temp_xlsb     0x24
// 8: hum_msb       0x25
// 9: hum_lsb       0x26
// 10: _            0x27
// 11: _            0x28
// 12: _            0x29
// 13: gas_r_msb    0x2A
// 14: gas_r_lsb    0x2B
bitfield! {
    pub struct RawData([u8]);
    impl Debug;

    pub u8, gas_range, _: calc_position(3, 14), calc_position(0, 14);
    // Each measuring cycle contains a  gas measurment slot, either a real one or a dummy one.
    // gas_valid indicates wether a real gas conversion (i.e. not a dummy one) is returned.
    pub bool, gas_valid, _: calc_position(5, 14);
    // Indicates if the heater target temperature was reached
    pub bool, heater_sable, _: calc_position(4, 14);
    pub u16, from into GasADC, gas_adc, _: calc_position(7, 14), calc_position(0, 13);
    pub u16, from into Humidity, humidity_adc, _: calc_position(7, 9), calc_position(0, 8);
    pub u32, from into Measurment, temperature_adc, _: calc_position(7, 7), calc_position(0, 5);
    pub u32, from into Measurment, pressure_adc, _: calc_position(7, 4), calc_position(0, 2);
    // measurment status
    // up to 10 conversions numbered from 0 to 9
    pub u8, gas_meas_index, _: calc_position(3, 0), calc_position(0, 0);
    // true if measuring is not done yet
    pub bool, measuring, _: calc_position(5, 0);
    // true if gas measuring is not done yet
    pub bool, gas_measuring, _: calc_position(6, 0);
    // true if data is available
    pub bool, new_data, _: calc_position(7, 0);
}

/// Temperature/Pressure adc values. 20 bits consisting of msb, lsb, xlsb
#[derive(Debug)]
pub struct Measurment(pub u32);
impl From<u32> for Measurment {
    fn from(value: u32) -> Self {
        let measurment_value = u32::from_be(value) >> 12;
        Measurment(measurment_value)
    }
}

/// Humidity adc value. 16 bits
#[derive(Debug)]
pub struct Humidity(pub u16);
impl From<u16> for Humidity {
    fn from(value: u16) -> Self {
        // switch bytes around
        let humidity = u16::from_be(value);
        Humidity(humidity)
    }
}

/// gas adc value. 10 bits
#[derive(Debug)]
pub struct GasADC(pub u16);
impl From<u16> for GasADC {
    fn from(value: u16) -> Self {
        // bit 7/6 from
        let [gas_r_msb, gas_r_lsb] = value.to_le_bytes();
        let mut gas_adc: u16 = gas_r_msb.into();
        // make space for 2 low bits
        gas_adc <<= 2;
        gas_adc |= (gas_r_lsb >> 6) as u16;
        GasADC(gas_adc)
    }
}

// used to calculate the actual bit position in bitfields with multiple bytes
fn calc_position(bit_position: usize, byte_offset: usize) -> usize {
    byte_offset * 8 + bit_position
}
#[cfg(test)]
mod tests {
    extern crate std;
    use crate::config::Configuration;
    use std::println;

    use super::{calc_position, Humidity, Measurment, RawConfig, RawData};
    use bitfield::bitfield;

    bitfield! {
        pub struct SampleData([u8]);
        impl Debug;
        pub u32, from into Measurment, m, _: calc_position(7, 2), calc_position(0, 0);
        pub u16, from into Humidity, h, _: calc_position(7,4), calc_position(0, 3);
    }

    #[test]
    fn test_raw_data() {
        let data = [
            // new_data, gas_measuring, measuring, _, gas_meas_index
            0b1_0_0_0_0000u8,
            // placeholder
            0,
            // p_msb
            0b00101001,
            // p_lsb
            0b10110011,
            // p_xlsb
            0b1111_0000,
            // t_msb
            0b00101001,
            // t_lsb
            0b10110011,
            // t_xlsb
            0b1111_0000,
            // h_msb
            0b10110011,
            // h_lsb
            0b00101001,
            // 3 placeholders
            0,
            0,
            0,
            // gas_r_msb gas_adc<9:2>
            0b10000001,
            // gas_r_lsb gas_adc<1:0>, gas_valid, heater_stable, gas_range
            0b11_1_1_0011,
        ];
        let expected_new_data = true;
        let expected_gas_measuring = false;
        let expected_measuring = false;
        let expected_gas_meas_index = 0u8;
        let expected_pressure = 0b0000_00000000_00101001_10110011_1111_u32;
        let expected_temperature = 0b0000_00000000_00101001_10110011_1111_u32;
        let expected_humidity = 0b10110011_00101001_u16;
        let expected_gas_adc = 0b10000001_11u16;
        let expected_gas_range = 0b011u8;
        let expected_gas_valid = true;
        let expected_heater_stable = true;
        let raw_data = RawData(data);
        assert!(raw_data.new_data() == expected_new_data);
        assert!(raw_data.gas_measuring() == expected_gas_measuring);
        assert!(raw_data.measuring() == expected_measuring);
        assert!(raw_data.gas_meas_index() == expected_gas_meas_index);
        assert!(raw_data.pressure_adc().0 == expected_pressure);
        assert!(raw_data.temperature_adc().0 == expected_temperature);
        assert!(raw_data.humidity_adc().0 == expected_humidity);
        assert!(raw_data.gas_adc().0 == expected_gas_adc);
        assert!(raw_data.gas_valid() == expected_gas_valid);
        assert!(raw_data.heater_sable() == expected_heater_stable);
        assert!(raw_data.gas_range() == expected_gas_range);
    }

    #[test]
    fn test_measurment_and_humidty() {
        // 2.5 bytes msb, lsb, xlsb 2 bytes msb, lsb
        let data = [0b00101001, 0b10110011, 0b1111_0000, 0b10110011, 0b00101001];
        let expected_measurment_value = 0b0000_00000000_00101001_10110011_1111_u32;
        let expected_humidity = 0b10110011_00101001_u16;
        let measurment = SampleData(data);
        assert!(expected_measurment_value == measurment.m().0);
        assert!(expected_humidity == measurment.h().0);
    }
    #[test]
    fn test_assemble() {
        let mut result = 0;
        let xlsb: u32 = 0b1111_0000;
        let lsb = 0b10110011;
        let msb = 0b00101001;
        let expected_value = 0b101001_10110011_1111;
        result |= xlsb >> 4;
        println!("Adding xlsb {xlsb:b} results in: {result:b}");
        result |= lsb << 4;
        println!("Adding lsb {lsb:b} results in: {result:b}");
        result |= msb << 12;
        println!("Adding msb {msb:b} results in: {result:b}");
        assert!(result == expected_value);
    }
    #[test]
    fn test_raw_config() {
        let mut raw_config = RawConfig([0u8; 5]);
        let default_user_config = Configuration::default();
        raw_config.apply_config(&default_user_config);
        let raw_data = raw_config.0;
        let expected_raw_data = [
            // 0x71 run_gas/nb_conv
            0b_000_1_0000u8,
            // 0x72 humidity oversampling by 16
            0b_0_0_000_101,
            // 0x73 placeholder
            0b0,
            // 0x74 temperature oversampling by 2 / pressure oversampling by 1 / mode sleep
            0b_010_001_00,
            // 0x75 filter coeff 1
            0b000_001_00,
        ];
        println!("Expeced data: {expected_raw_data:?}");
        println!("Actual raw data: {raw_data:?}");
        assert!(expected_raw_data == raw_data);
    }
}
