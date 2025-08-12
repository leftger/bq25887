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

const LARGEST_REG_SIZE_BYTES: usize = 0x01;
const LARGEST_REG_SIZE_BITS: u32 = 0x08;

const CHARGE_CURRENT_LIMIT_ADDR:u8 = 0x01; 
const INPUT_VOLTAGE_LIMIT_ADDR:u8 = 0x02; 


const BQ_ADDR: u8 = 0x6A;

device_driver::create_device!(
    device_name: Bq25887,
    manifest: "src/bq25887.yaml"
);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BQ25887Error<I2cError> {
    I2c(I2cError),
    Conversion(device_driver::ConversionError<u8>),
}

impl<E> From<device_driver::ConversionError<u8>> for BQ25887Error<E> {
    fn from(err: device_driver::ConversionError<u8>) -> Self {
        BQ25887Error::Conversion(err)
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
    device: Bq25887<DeviceInterface<I2C>>,
}

/// Cell Voltage Regulation Limit Register BQ25887 p.33
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VoltageRegulationLimit {
    pub vcellreg: u8, //todo: Should create millivolt helper impl
}

/// Charger Current Limit Register BQ25887 p.33
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChargeCurrentLimit {
    pub en_hiz: bool,
    pub en_ilim: bool,
    pub ichg: u8, // todo: 6-bit value (0x00–0x3F), should be clamped by user
}

impl From<u8> for ChargeCurrentLimit {
    fn from(byte: u8) -> Self {
        Self {
            en_hiz: (byte & 0b1000_0000) != 0,
            en_ilim: (byte & 0b0100_0000) != 0,
            ichg: byte & 0b0011_1111,
        }
    }
}

impl From<ChargeCurrentLimit> for u8 {
    fn from(val: ChargeCurrentLimit) -> Self {
        let hiz = if val.en_hiz { 1 << 7 } else { 0 };
        let ilim = if val.en_ilim { 1 << 6 } else { 0 };
        let ichg = val.ichg & 0b0011_1111;
        hiz | ilim | ichg
    }
}

/// Input Voltage Limit Register BQ25887 p.35
pub struct InputVoltageLimit{
    pub en_vindpm_rst: bool,
    pub en_bat_dischg: bool,
    pub pfm_ooa_dis: bool,
    pub vindpm: u8, //todo: 5 bit value (0x00-0x1F), should be clamped by user
}

impl From<u8> for InputVoltageLimit {
    fn from(byte: u8) -> Self {
        Self {
            en_vindpm_rst: (byte & 0b1000_0000) != 0,
            en_bat_dischg: (byte & 0b0100_0000) != 0,
            pfm_ooa_dis: (byte & 0b0010_0000) != 0,
            vindpm: byte & 0b0001_1111,
        }
    }
}

impl From<InputVoltageLimit> for u8 {
    fn from(val: InputVoltageLimit) -> Self {
        let en_vindpm_rst = if val.en_vindpm_rst { 1 << 7 } else { 0 };
        let en_bat_dischg = if val.en_bat_dischg { 1 << 6 } else { 0 };
        let pfm_ooa_dis = if val.pfm_ooa_dis { 1 << 5 } else { 0 };
        let vindpm = val.vindpm & 0b0001_1111;
        en_vindpm_rst | en_bat_dischg | pfm_ooa_dis | vindpm
    }
}

/// Input Current Limit Register BQ25887 p.36
pub struct InputCurrentLimit{
    pub force_ico: bool,
    pub force_indet: bool,
    pub en_ico: bool,
    pub iindpm: u8, //todo: 5 bit value (0x00-0x1F), should be clamped by user
}

impl<I2C: I2cTrait> Bq25887Driver<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Bq25887Driver {
            device: Bq25887::new(DeviceInterface { i2c }),
        }
    }

    /// ### Breif
    /// Reads the Cell Voltage Regulation Limit Register,
    /// (Address = 0x00) (reset = 0xA0),
    /// BQ25887 p.33
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_voltage_regulation_limit(&mut self) -> Result<VoltageRegulationLimit, BQ25887Error<I2C::Error>> {
        let cell_voltage_limit = self.device.cell_voltage_limit().read_async().await?;
        let return_val = VoltageRegulationLimit {
            vcellreg: cell_voltage_limit.vcellreg(),
        };
        Ok(return_val)
    }

    /// ### Breif
    /// Writes the given `VoltageRegulationLimit` to the Cell Voltage Regulation Limit Register,
    /// (Address = 0x00) (reset = 0xA0),
    /// BQ25887 p.33
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_voltage_regulation_limit(
        &mut self,
        volt_limit: VoltageRegulationLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        let raw_register_value: u8 = volt_limit.vcellreg;
        self.device
            .cell_voltage_limit()
            .write_async(|reg| reg.set_vcellreg(raw_register_value))
            .await
    }

    /// ### Breif
    /// Reads Charger Current Limit Register,
    /// (Address = 0x01) (reset = 0x5E),
    /// BQ25887 p.34
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_charge_current_limit(&mut self) -> Result<ChargeCurrentLimit, BQ25887Error<I2C::Error>> {
        let reg = self.device.charge_current_limit().read_async().await?;
        let en_hiz = u8::from(reg.en_hiz()) << 7;
        let en_ilim = u8::from(reg.en_ilim()) << 6;
        let ichg: u8 = reg.ichg()?.into();

        let byte = (en_hiz) | (en_ilim) | (ichg & 0b0011_1111);

        Ok(byte.into())
    }

    /// ### Breif
    /// Writes to the Charger Current Limit Register,
    /// (Address = 0x01) (reset = 0x5E),
    /// BQ25887 p.34
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charge_current_limit(&mut self, current_limit: ChargeCurrentLimit) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        let raw_register_value:u8 = current_limit.into();
        buf[0] = raw_register_value;
        self.device.interface().write_register(CHARGE_CURRENT_LIMIT_ADDR, LARGEST_REG_SIZE_BITS, &buf).await?;
        Ok(())
    }

    /// ### Breif
    /// Reads Input Voltage Limit Register,
    /// (Address = 02h) (reset = 84h),
    /// BQ25887 p.35
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_input_voltage_limit(&mut self) -> Result<InputVoltageLimit, BQ25887Error<I2C::Error>> {
        let reg = self.device.input_voltage_limit().read_async().await?;
        let en_vindpm_rst = u8::from(reg.en_vindpm_rst()) << 7;
        let en_bat_dischg = u8::from(reg.en_bat_dischg()) << 6;
        let pfm_ooa_dis = u8::from(reg.pfm_ooa_dis()) << 5;
        let vindpm:u8 = reg.vindpm()?.into();

        let byte = en_vindpm_rst | en_bat_dischg | pfm_ooa_dis | (vindpm & 0b0001_1111);
        Ok(byte.into())
    }

    /// ### Breif
    /// Writes Input Voltage Limit Register,
    /// (Address = 02h) (reset = 84h),
    /// BQ25887 p.35
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_input_voltage_limit(&mut self, input_volt_limit: InputVoltageLimit) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        let raw_register_value:u8 = input_volt_limit.into();
        buf[0] = raw_register_value;
        self.device.interface().write_register(INPUT_VOLTAGE_LIMIT_ADDR, LARGEST_REG_SIZE_BITS, &buf).await?;
        Ok(())
    }


    // pub async fn read_charger_status_1(&mut self) -> Result<u8, BQ25887Error<I2C::Error>> {
    //     let reg = self.device.charger_status_1().read_async().await?;
    //     let iindpm = u8::from(reg.iindpm_stat()) << 6;
    //     let vindpm = u8::from(reg.vindpm_stat()) << 5;
    //     let treg = u8::from(reg.treg_stat()) << 4;
    //     let wd_stat = u8::from(reg.wd_stat()) << 3;
    //     let charge_stat: u8 = reg.chrg_stat().into();
    //     let byte = iindpm | vindpm | treg | wd_stat | (charge_stat & 0b111);
    //     Ok(byte)
    // }
}
