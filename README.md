# bosch_bme680 &emsp; [![crates.io](https://img.shields.io/crates/v/bosch-bme680)](https://crates.io/crates/bosch-bme680)
*A pure rust driver for the Bosch BME680 environmental sensor that focuses on ease of use.*


## Additional information
- [BME680 product page](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/)
- Official [C version](https://github.com/BoschSensortec/BME680_driver)
- More detailed documentation can be found in the download section of the product page.

## Simple example
Mock i2c and delay have to be replaced with specific hardware crates.

```rust
fn main() -> ! {
    let i2c = mock::blocking_i2c();
    let delay = mock::MockDelay;

    let config = bosch_bme680::Configuration::default();
    let mut bme = Bme680::new(i2c, DeviceAddress::Primary, delay, &config, 20).unwrap();
    thread::sleep(Duration::from_millis(100));

    loop {
        thread::sleep(Duration::from_secs(2));
        let values = bme.measure().unwrap();
        println!("Values: {values:?}\n");
    }
}
```

## [`embedded-hal-async`] usage

This crate has optional support for [`embedded-hal-async`], which provides
asynchronous versions of the [`embedded-hal`] traits. To avoid an unnecessary
dependency on `embedded-hal-async` for projects which do not require it, the
`embedded-hal-async` support is an optional feature.

In order to use the `embedded-hal-async` driver, add the following to your
`Cargo.toml`:

```toml
[dependencies]
bosch-bme680 = { version = "1.0.3", features = ["embedded-hal-async"] }
```

Then, construct an instance of the `AsyncBme680` struct using the
`embedded_hal_async` `I2c` and `Delay` traits.

[`embedded-hal`]: https://crates.io/crates/embedded-hal-async
[`embedded-hal-async`]: https://crates.io/crates/embedded-hal-async
