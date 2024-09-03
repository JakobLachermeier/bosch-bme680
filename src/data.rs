#[derive(Debug)]
pub struct CalibrationData {
    // Temperature coefficients
    pub par_t1: u16,
    pub par_t2: i16,
    pub par_t3: i8,
    // Pressure coefficients
    pub par_p1: u16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i16,
    pub par_p5: i16,
    pub par_p6: i8,
    pub par_p7: i8,
    pub par_p8: i16,
    pub par_p9: i16,
    pub par_p10: u8,
    // Humidity coefficients
    pub par_h1: u16,
    pub par_h2: u16,
    pub par_h3: i8,
    pub par_h4: i8,
    pub par_h5: i8,
    pub par_h6: u8,
    pub par_h7: i8,
    // Gas heater
    pub par_gh1: i8,
    pub par_gh2: i16,
    pub par_gh3: i8,

    // Other
    pub res_heat_range: u8,
    pub res_heat_val: i8,
    pub range_sw_err: i8,
}

/// Measurment data returned from the sensor
#[derive(Debug)]
pub struct MeasurmentData {
    /// Temperature in °C
    pub temperature: f32,
    /// Relative humidity in %
    pub humidity: f32,
    /// Pressure in hPa
    pub pressure: f32,
    /// Gas resistance in Ohms
    /// None if gas measurment is disabled or gas measurment hasn't finished in time according to the gas_measuring bit.
    pub gas_resistance: Option<f32>,
}

impl MeasurmentData {
    pub(crate) fn from_raw(
        raw_data: crate::bitfields::RawData<[u8; 15]>,
        calibration_data: &CalibrationData,
        variant: &crate::config::Variant,
    ) -> Option<Self> {
        // First, check to make sure a measurement is ready. If not, bail.
        if raw_data.measuring() && !raw_data.new_data() {
            return None;
        }
        let (temperature, t_fine) =
            crate::calculate_temperature(raw_data.temperature_adc().0, calibration_data);
        let pressure =
            crate::calculate_pressure(raw_data.pressure_adc().0, calibration_data, t_fine);
        let humidity =
            crate::calculate_humidity(raw_data.humidity_adc().0, calibration_data, t_fine);
        let gas_resistance = if raw_data.gas_valid() && !raw_data.gas_measuring() {
            let gas_resistance = variant.calc_gas_resistance(
                raw_data.gas_adc().0,
                calibration_data.range_sw_err,
                raw_data.gas_range() as usize,
            );
            Some(gas_resistance)
        } else {
            None
        };

        Some(MeasurmentData {
            temperature,
            gas_resistance,
            humidity,
            pressure,
        })
    }
}

pub fn calculate_temperature(adc_temp: u32, calibration_data: &CalibrationData) -> (f32, f32) {
    let temp_adc = adc_temp as f32;
    let var_1 = ((temp_adc / 16384.) - (calibration_data.par_t1 as f32 / 1024.))
        * calibration_data.par_t2 as f32;
    let var_2 = (((temp_adc / 131072.) - (calibration_data.par_t1 as f32 / 8192.))
        * ((temp_adc / 131072.) - (calibration_data.par_t1 as f32 / 8192.)))
        * (calibration_data.par_t3 as f32 * 16.);
    // store for use in pressure and hummidity calculation
    let t_fine = var_1 + var_2;
    let calc_temp = t_fine / 5120.;
    (calc_temp, t_fine)
}

pub fn calculate_pressure(adc_press: u32, calibration_data: &CalibrationData, t_fine: f32) -> f32 {
    let adc_press = adc_press as f32;
    let var1 = (t_fine / 2.) - 64000.;
    let var2 = var1 * var1 * (calibration_data.par_p6 as f32 / 131072.);
    let var2 = var2 + (var1 * calibration_data.par_p5 as f32 * 2.);
    let var2 = (var2 / 4.) + (calibration_data.par_p4 as f32 * 65536.);
    let var1 = (((calibration_data.par_p3 as f32 * var1 * var1) / 16384.)
        + (calibration_data.par_p2 as f32 * var1))
        / 524288.;
    let var1 = (1. + (var1 / 32768.)) * calibration_data.par_p1 as f32;
    let mut calc_pres = 1048576. - adc_press;
    if var1 as i16 != 0 {
        calc_pres = ((calc_pres - (var2 / 4096.)) * 6250.) / var1;
        let var1 = (calibration_data.par_p9 as f32 * calc_pres * calc_pres) / 2147483648.;
        let var2 = calc_pres * (calibration_data.par_p8 as f32 / 32768.);
        let var3 = (calc_pres / 256.)
            * (calc_pres / 256.)
            * (calc_pres / 256.)
            * (calibration_data.par_p10 as f32 / 131072.);
        calc_pres += (var1 + var2 + var3 + (calibration_data.par_p7 as f32 * 128.)) / 16.;
    } else {
        calc_pres = 0.;
    }
    calc_pres
}

pub fn calculate_humidity(adc_hum: u16, calibration_data: &CalibrationData, t_fine: f32) -> f32 {
    let adc_hum = adc_hum as f32;
    let temp_comp = t_fine / 5120.;
    let var1 = (adc_hum)
        - ((calibration_data.par_h1 as f32 * 16.)
            + ((calibration_data.par_h3 as f32 / 2.) * temp_comp));
    let var2 = var1
        * ((calibration_data.par_h2 as f32 / 262144.)
            * (1.
                + ((calibration_data.par_h4 as f32 / 16384.) * temp_comp)
                + ((calibration_data.par_h5 as f32 / 1048576.) * temp_comp * temp_comp)));
    let var3 = calibration_data.par_h6 as f32 / 16384.;
    let var4 = calibration_data.par_h7 as f32 / 2097152.;
    let mut calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
    calc_hum = calc_hum.clamp(0., 100.);
    // Reference implemetation uses this.
    // if calc_hum > 100. {
    //     calc_hum = 100.;
    // } else if calc_hum < 0. {
    //     calc_hum = 0.
    // }
    calc_hum
}

#[cfg(test)]
mod tests {
    use crate::data::{
        calculate_humidity, calculate_pressure, calculate_temperature, CalibrationData,
    };
    use approx::{assert_abs_diff_eq, assert_relative_eq, relative_eq};

    static CALIBRATION_DATA: CalibrationData = CalibrationData {
        par_t1: 25942,
        par_t2: 26664,
        par_t3: 3,
        par_p1: 37439,
        par_p2: -10316,
        par_p3: 88,
        par_p4: 10477,
        par_p5: -308,
        par_p6: 30,
        par_p7: 62,
        par_p8: -5160,
        par_p9: -1568,
        par_p10: 30,
        par_h1: 881,
        par_h2: 989,
        par_h3: 0,
        par_h4: 45,
        par_h5: 20,
        par_h6: 120,
        par_h7: -100,
        par_gh1: -69,
        par_gh2: -9092,
        par_gh3: 18,
        res_heat_range: 1,
        res_heat_val: 30,
        range_sw_err: 0,
    };

    #[test]
    fn test_calc_temp() {
        // Calc_temp: temp_adc: 482062, calc_temp: 21.295866, tfine: 109034.835938
        // Calc_temp: temp_adc: 482452, calc_temp: 21.419861, tfine: 109669.687500
        // Calc_temp: temp_adc: 482060, calc_temp: 21.295231, tfine: 109031.585938
        // Calc_temp: temp_adc: 482453, calc_temp: 21.420179, tfine: 109671.320312
        // Calc_temp: temp_adc: 482058, calc_temp: 21.294596, tfine: 109028.328125

        let data = [
            (482062, 21.295866, 109034.835938),
            (482452, 21.419861, 109669.687500),
            (482060, 21.295231, 109031.585938),
            (482453, 21.420179, 109671.320312),
            (482058, 21.294596, 109028.328125),
        ];
        for (temp_adc, actual_temp, actual_tfine) in data {
            let (temp, calc_tfine) = calculate_temperature(temp_adc, &CALIBRATION_DATA);

            assert_abs_diff_eq!(actual_temp, temp);
            assert_abs_diff_eq!(actual_tfine, calc_tfine);
        }
    }
    #[test]
    fn test_calc_humidity() {
        // hum_adc: 25537, calc_hum: 59.469585, tfine: 109842.234375
        // hum_adc: 25531, calc_hum: 59.410557, tfine: 109090.187500
        // hum_adc: 25545, calc_hum: 59.515030, tfine: 109643.640625
        // hum_adc: 25535, calc_hum: 59.431923, tfine: 108942.054688
        // hum_adc: 25549, calc_hum: 59.537392, tfine: 109531.328125

        let pairs = [
            (25537, 59.469585, 109842.234375),
            (25531, 59.410557, 109090.187500),
            (25545, 59.515030, 109643.640625),
            (25535, 59.431923, 108942.054688),
            (25549, 59.537392, 109531.328125),
        ];
        for (hum_adc, actual_hum, tfine) in pairs {
            let calc_hum = calculate_humidity(hum_adc, &CALIBRATION_DATA, tfine);
            assert_abs_diff_eq!(calc_hum, actual_hum);
        }
    }
    #[test]
    fn test_calc_pressure() {
        // pres_adc: 307582, calc_pres: 95058.664062, tfine: 111095.656250
        // pres_adc: 307395, calc_pres: 95058.992188, tfine: 110130.359375
        // pres_adc: 307469, calc_pres: 95059.296875, tfine: 110525.921875
        // pres_adc: 307313, calc_pres: 95058.773438, tfine: 109695.726562
        // pres_adc: 307254, calc_pres: 95060.328125, tfine: 109436.914062
        let data = [
            (307582, 95058.664062, 111095.656250),
            (307395, 95058.992188, 110130.359375),
            (307469, 95059.296875, 110525.921875),
            (307313, 95058.773438, 109695.726562),
            (307254, 95060.328125, 109436.914062),
        ];
        for (press_adc, actual_press, tfine) in data {
            let calc_press = calculate_pressure(press_adc, &CALIBRATION_DATA, tfine);
            assert_abs_diff_eq!(calc_press, actual_press);
        }
    }
}
