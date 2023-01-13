use core::fmt::Formatter;

use embedded_hal::blocking::i2c::{Write, WriteRead};

pub enum BmeError<I2C>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
{
    WriteError(<I2C as Write>::Error),
    WriteReadError(<I2C as WriteRead>::Error),
    UnexpectedChipId(u8),
    UnexpectedSensorStatus(u8),
    MeasuringTimeOut,
}

impl<I2C> core::fmt::Debug for BmeError<I2C>
where
    I2C: WriteRead + Write,
    <I2C as WriteRead>::Error: core::fmt::Debug,
    <I2C as Write>::Error: core::fmt::Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::result::Result<(), core::fmt::Error> {
        match self {
            BmeError::WriteReadError(e) => f.debug_tuple("WriteReadError").field(e).finish(),
            BmeError::WriteError(e) => f.debug_tuple("WriteError").field(e).finish(),
            BmeError::UnexpectedChipId(chip_id) => f
                .debug_tuple("Got unimplemented chip id: ")
                .field(chip_id)
                .finish(),
            BmeError::UnexpectedSensorStatus(value) => f
                .debug_tuple("Got incorrect status from control register. Valid values are 0 or 1.")
                .field(value)
                .finish(),
            BmeError::MeasuringTimeOut => f
                .debug_tuple("Timed out while waiting for new measurment values. Either no new data or the sensor took unexpectedly long to finish measuring.").finish()
        }
    }
}
