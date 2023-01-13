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

#[derive(Debug)]
pub struct MeasurmentData {
    pub temperature: f32,
    pub humidity: f32,
    pub pressure: f32,
    pub gas_resistance: Option<f32>,
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
