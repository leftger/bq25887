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

use core::cell::Cell;
#[cfg(feature = "defmt")]
use defmt::{Format, Formatter, write};
use device_driver::AsyncRegisterInterface;
use embedded_hal_async::i2c::I2c as I2cTrait;

const LARGEST_REG_SIZE_BYTES: usize = 1;

const BQ_ADDR: u8 = 0x6A;

device_driver::create_device!(
    device_name: Bq25887,
    manifest: "src/bq25887.yaml"
);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum BQ25887Error<I2cError> {
    I2c(I2cError),
    Conversion(device_driver::ConversionError<u8>),
}

#[cfg(feature = "defmt")]
impl<I2cError: defmt::Format> defmt::Format for BQ25887Error<I2cError> {
    fn format(&self, f: defmt::Formatter) {
        match self {
            BQ25887Error::I2c(err) => defmt::write!(f, "I2C error: {}", err),
            BQ25887Error::Conversion(err) => defmt::write!(f, "Conversion error: {}", err),
        }
    }
}

impl<I2C: I2cTrait> device_driver::AsyncRegisterInterface for DeviceInterface<I2C> {
    type Error = BQ25887Error<I2C::Error>;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + LARGEST_REG_SIZE_BYTES];
        buf[0] = address;
        buf[1..=data.len()].copy_from_slice(data);
        self.i2c
            .write(BQ_ADDR, &buf[..=data.len()])
            .await
            .map_err(BQ25887Error::I2c)
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c
            .write_read(BQ_ADDR, &[address], data)
            .await
            .map_err(BQ25887Error::I2c)
    }
}

/// BQ25887 interface, which takes an async I2C bus
pub struct DeviceInterface<I2C: I2cTrait> {
    pub i2c: I2C,
}

pub struct Bq25887Driver<I2C: I2cTrait> {
    device: Bq25887<DeviceInterface<I2C>>
}

impl<E> From<device_driver::ConversionError<u8>> for BQ25887Error<E> {
    fn from(err: device_driver::ConversionError<u8>) -> Self {
        BQ25887Error::Conversion(err)
    }
}

impl<I2C: I2cTrait> Bq25887Driver<I2C> {
    /// Create a new BQ25887 driver instance
    pub fn new(i2c: I2C) -> Self {
        Bq25887Driver {
            device: Bq25887::new(DeviceInterface { i2c })
        }
    }

    // Cell Voltage Regulation Limit Register (Address = 00h) [reset = A0h] p.33

    /// Reads the Voltage Regulation Limit Register
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_voltage_regulation_limit(&mut self) -> Result<u8, BQ25887Error<I2C::Error>> {
        let cell_voltage_limit = self.device.cell_voltage_limit().read_async().await?;
        let vcellreg_val = cell_voltage_limit.vcellreg();
        Ok(vcellreg_val)
    }

    /// Writes the Voltage Regulation Limit Register (u8)
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn write_voltage_regulation_limit(&mut self, voltage_mv: u8) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .cell_voltage_limit()
            .write_async(|reg| reg.set_vcellreg(voltage_mv))
            .await
    }

    ///Charger Current Limit Register (Address = 01h) [reset = 5Eh]
    /// p.34
    pub async fn read_charge_current_limit(&mut self) -> Result<u8, BQ25887Error<I2C::Error>> {
        // let mut buf = [0u8; 1];
        // self.device
        //     .interface()
        //     .read_register(0x01, 8, &mut buf)
        //     .await?;
        // Ok(buf[0])
        let reg = self.device.charge_current_limit().read_async().await?;
        let en_hiz = u8::from(reg.en_hiz()) << 7;
        let en_ilim = u8::from(reg.en_ilim()) << 6;
        let ichg: u8 = reg.ichg()?.into();

        // Assemble full byte
        // EN_HIZ = bit 7
        // EN_ILIM = bit 6
        // ICHG = bits 5:0 (should already be clamped)
        let byte = (en_hiz) | (en_ilim) | (ichg & 0x3F);
        Ok(byte)
    }

    pub async fn write_charge_current_limit(&mut self, current: u8) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        buf[0] = current;
        self.device.interface().write_register(0x01, 8, &buf).await?;
        Ok(())
    }

    pub async fn read_charger_status_1(&mut self) -> Result<u8, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_status_1().read_async().await?;
        let iindpm = u8::from(reg.iindpm_stat()) << 6;
        let vindpm = u8::from(reg.vindpm_stat()) << 5;
        let treg = u8::from(reg.treg_stat()) << 4;
        let wd_stat = u8::from(reg.wd_stat()) << 3;
        let charge_stat: u8 = reg.chrg_stat().into();
        let byte = iindpm | vindpm | treg | wd_stat | (charge_stat & 0b111);
        Ok(byte)
    }
}
