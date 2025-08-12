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
#![allow(missing_docs)]
#[cfg(feature = "defmt-03")]
use defmt::{Format, Formatter, write};
use device_driver::AsyncRegisterInterface;
use embedded_hal_async::i2c::I2c as I2cTrait;
use modular_bitfield::prelude::*;

const LARGEST_REG_SIZE_BYTES: usize = 0x01;
const LARGEST_REG_SIZE_BITS: u32 = 0x08;

const CHARGE_CURRENT_LIMIT_ADDR: u8 = 0x01;
const INPUT_VOLTAGE_LIMIT_ADDR: u8 = 0x02;
const INPUT_CURRENT_LIMIT_ADDR: u8 = 0x03;
const PRECHARGE_AND_TERMINATION_CURRENT_ADDR: u8 = 0x04;

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

#[bitfield]
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VoltageRegulationLimit {
    pub vcellreg: B8, //todo: Should create millivolt helper impl
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargeCurrentLimit {
    pub ichg: B6,
    pub en_ilim: B1,
    pub en_hiz: B1,
}

impl From<u8> for ChargeCurrentLimit {
    fn from(byte: u8) -> Self {
        let mut buf = [0u8; 1];
        buf[0] = byte;
        Self { bytes: buf }
    }
}

impl From<ChargeCurrentLimit> for u8 {
    fn from(val: ChargeCurrentLimit) -> Self {
        let hiz = if val.en_hiz() != 0 { 1 << 7 } else { 0 };
        let ilim = if val.en_ilim() != 0 { 1 << 6 } else { 0 };
        let ichg = val.ichg() & 0b0011_1111;
        hiz | ilim | ichg
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct InputVoltageLimit {
    pub vindpm: B5,
    pub pfm_ooa_dis: B1,
    pub en_bat_dischg: B1,
    pub en_vindpm_rst: B1,
}

impl From<u8> for InputVoltageLimit {
    fn from(byte: u8) -> Self {
        let mut buf = [0u8; 1];
        buf[0] = byte;
        Self { bytes: buf }
    }
}

impl From<InputVoltageLimit> for u8 {
    fn from(val: InputVoltageLimit) -> Self {
        let en_vindpm_rst = if val.en_vindpm_rst() != 0 { 1 << 7 } else { 0 };
        let en_bat_dischg = if val.en_bat_dischg() != 0 { 1 << 6 } else { 0 };
        let pfm_ooa_dis = if val.pfm_ooa_dis() != 0 { 1 << 5 } else { 0 };
        let vindpm = val.vindpm() & 0b0001_1111;
        en_vindpm_rst | en_bat_dischg | pfm_ooa_dis | vindpm
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct InputCurrentLimit {
    pub iindpm: B5,
    pub en_ico: B1,
    pub force_indet: B1,
    pub force_ico: B1,
}

impl From<u8> for InputCurrentLimit {
    fn from(byte: u8) -> Self {
        let mut buf = [0u8; 1];
        buf[0] = byte;
        Self { bytes: buf }
    }
}

impl From<InputCurrentLimit> for u8 {
    fn from(value: InputCurrentLimit) -> Self {
        let force_ico = if value.force_ico() != 0 { 1 << 7 } else { 0 };
        let force_indet = if value.force_indet() != 0 { 1 << 6 } else { 0 };
        let en_ico = if value.en_ico() != 0 { 1 << 5 } else { 0 };
        let iindpm = value.iindpm() & 0b0001_1111;
        force_ico | force_indet | en_ico | iindpm
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct PrechargeAndTerminationCurrentLimit {
    pub iterm: B4,
    pub iprechg: B4,
    
}

impl From<u8> for PrechargeAndTerminationCurrentLimit {
    fn from(byte: u8) -> Self {
        let mut buf = [0u8; 1];
        buf[0] = byte;
        Self { bytes: buf }
    }
}

impl From<PrechargeAndTerminationCurrentLimit> for u8 {
    fn from(value: PrechargeAndTerminationCurrentLimit) -> Self {
        let iprechg = (value.iprechg() & 0b0000_1111) << 4;
        let iterm = value.iterm() & 0b0000_1111;
        iprechg | iterm
    }
}

// ------------------------------------------------------------------------------------

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
        let regulation_limit = VoltageRegulationLimit::new().with_vcellreg(cell_voltage_limit.vcellreg());
        Ok(regulation_limit)
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
        let raw_register_value: u8 = volt_limit.vcellreg();
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
    pub async fn write_charge_current_limit(
        &mut self,
        current_limit: ChargeCurrentLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = current_limit.into();
        self.device
            .interface()
            .write_register(CHARGE_CURRENT_LIMIT_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
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
        let vindpm: u8 = reg.vindpm()?.into();

        let byte = en_vindpm_rst | en_bat_dischg | pfm_ooa_dis | (vindpm & 0b0001_1111);
        Ok(byte.into())
    }

    /// ### Breif
    /// Writes Input Voltage Limit Register,
    /// (Address = 02h) (reset = 84h),
    /// BQ25887 p.35
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_input_voltage_limit(
        &mut self,
        input_volt_limit: InputVoltageLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = input_volt_limit.into();
        self.device
            .interface()
            .write_register(INPUT_VOLTAGE_LIMIT_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }

    /// ### Breif
    /// Reads Input Current Limit Register,
    /// (Address = 0x03) (reset = 0x39 )
    /// BQ255887 p.36
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_input_current_limit(&mut self) -> Result<InputCurrentLimit, BQ25887Error<I2C::Error>> {
        let reg = self.device.input_current_limit().read_async().await?;
        let force_ico = u8::from(reg.force_ico()) << 7;
        let force_indet = u8::from(reg.force_indet()) << 6;
        let en_ico = u8::from(reg.en_ico()) << 5;
        let iindpm: u8 = reg.iindpm()?.into();

        let byte = force_ico | force_indet | en_ico | (iindpm & 0b0001_1111);
        Ok(byte.into())
    }

    /// ### Breif
    /// Writes Input Current Limit Register,
    /// (Address = 0x03) (reset = 0x39 )
    /// BQ255887 p.36
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_input_current_limit(
        &mut self,
        current_limit: InputCurrentLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = current_limit.into();
        self.device
            .interface()
            .write_register(INPUT_CURRENT_LIMIT_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }

    /// ### Breif
    /// Reads Precharge and Termination Current Limit Register,
    /// (Address = 0x04) [reset = 0x22]
    /// BQ255887 p.37
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_precharge_and_termination_current_limit(&mut self) -> Result<PrechargeAndTerminationCurrentLimit, BQ25887Error<I2C::Error>>{
        let reg = self.device.prechg_termination_ctrl().read_async().await?;
        let iprechg:u8 = reg.iprechg()?.into();
        let iterm:u8 = reg.iprechg()?.into();
        let byte = ((iprechg << 4) & 0b1111_0000) | (iterm & 0b0000_1111);
        Ok(byte.into())
    }

    /// ### Breif
    /// Writes Precharge and Termination Current Limit Register,
    /// (Address = 0x04) [reset = 0x22]
    /// BQ255887 p.37
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_precharge_and_termination_current_limit(&mut self, current_limit: PrechargeAndTerminationCurrentLimit) -> Result<(), BQ25887Error<I2C::Error>>{
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = current_limit.into();
        self.device
            .interface()
            .write_register(PRECHARGE_AND_TERMINATION_CURRENT_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }
}
