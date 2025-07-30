//! This is a platform-agnostic Rust driver for the Texas Instruments BQ25887 Battery
//! fuel/gas gauge based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! For further details of the device architecture and operation, please refer
//! to the official [`Datasheet`].
//!
//! [`Datasheet`]: https://www.ti.com/lit/ug/slusd89b/slusd89b.pdf

#![doc = include_str!("../README.md")]
#![cfg_attr(not(test), no_std)]
#![allow(missing_docs, warnings)]

// use core::cell::Cell;

// use embedded_batteries_async::smart_battery::{
//     self, BatteryModeFields, BatteryStatusFields, CapacityModeSignedValue, CapacityModeValue, DeciKelvin, ErrorCode,
//     SpecificationInfoFields,
// };
// use embedded_hal_async::i2c::I2c as I2cTrait;

// #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
// #[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
// pub enum BQ25887Error<I2cError> {
//     I2c(I2cError),
//     BatteryStatus(ErrorCode),
// }

const LARGEST_REG_SIZE_BYTES: usize = 1;
const LARGEST_CMD_SIZE_BYTES: usize = 1;
const LARGEST_BUF_SIZE_BYTES: usize = 1;

const BQ_ADDR: u8 = 0x6Au8;

// impl From<field_sets::BatteryStatus> for BatteryStatusFields {
//     fn from(value: field_sets::BatteryStatus) -> Self {
//         BatteryStatusFields::new()
//             .with_error_code(value.ec())
//     }
// }

device_driver::create_device!(
    device_name: Bq25887,
    manifest: "src/bq25887.yaml"
);

// impl<E: embedded_hal_async::i2c::Error> smart_battery::Error for BQ25887Error<E> {
//     fn kind(&self) -> smart_battery::ErrorKind {
//         match self {
//             Self::I2c(_) => smart_battery::ErrorKind::CommError,
//             Self::BatteryStatus(e) => smart_battery::ErrorKind::BatteryStatus(*e),
//         }
//     }
// }
