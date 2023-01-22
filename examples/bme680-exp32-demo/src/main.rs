use std::thread;
use std::time::Duration;

use bosch_bme680::Bme680;
use bosch_bme680::DeviceAddress;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::i2c::I2cConfig;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_hal::prelude::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

fn main() -> ! {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();
    let i2c = peripherals.i2c0;
    let i2c_config = I2cConfig::default().baudrate(400.kHz().into());
    let sda = peripherals.pins.gpio2;
    let scl = peripherals.pins.gpio3;
    let i2c_interface = I2cDriver::new(i2c, sda, scl, &i2c_config).unwrap();
    let config = bosch_bme680::Configuration::default();
    // Ets {} is used to create short delays between communication with the sensor.
    // The last parameter is the initial ambient temperature for humidity and pressure calculation.
    // It will be updated automatically with the measured temperature after the first measurment.
    let mut bme = Bme680::new(i2c_interface, DeviceAddress::Primary, Ets {}, &config, 20).unwrap();
    thread::sleep(Duration::from_millis(100));

    loop {
        thread::sleep(Duration::from_secs(2));
        let values = bme.measure().unwrap();
        println!("Values: {values:?}\n");
    }
}
