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

//todo change to enum
const CHARGE_CURRENT_LIMIT_ADDR: u8 = 0x01;
const INPUT_VOLTAGE_LIMIT_ADDR: u8 = 0x02;
const INPUT_CURRENT_LIMIT_ADDR: u8 = 0x03;
const PRECHARGE_AND_TERMINATION_CURRENT_ADDR: u8 = 0x04;
const CHARGER_CONTROL_1_ADDR: u8 = 0x05;
const CHARGER_CONTROL_2_ADDR: u8 = 0x06;
const CHARGER_CONTROL_3_ADDR: u8 = 0x07;
const CHARGER_CONTROL_4_ADDR: u8 = 0x08;
const CHARGER_MASK_1_ADDR: u8 = 0x12;
const CHARGER_MASK_2_ADDR: u8 = 0x13;

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

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct InputVoltageLimit {
    pub vindpm: B5,
    pub pfm_ooa_dis: B1,
    pub en_bat_dischg: B1,
    pub en_vindpm_rst: B1,
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

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct PrechargeAndTerminationCurrentLimit {
    pub iterm: B4,
    pub iprechg: B4,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerControl1 {
    pub tmr2x_en: B1,
    pub chg_timer: B2,
    pub en_timer: B1,
    pub watchdog: B2,
    pub stat_dis: B1,
    pub en_term: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerControl2 {
    pub vrechg: B2,
    pub celllowv: B1,
    pub en_chg: B1,
    pub treg: B2,
    pub auto_indet_en: B1,
    #[skip(setters, getters)]
    reseved: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerControl3 {
    #[skip(setters, getters)]
    reserved: B4,
    pub topoff_timer: B2,
    pub wd_rst: B1,
    pub pfm_dis: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerControl4 {
    pub jeita_isetc: B2,
    pub jeita_iseth: B1,
    pub jeita_vset: B2,
    #[skip(setters, getters)]
    reserved: B3,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct IcoCurrentLimitInUse {
    #[skip(setters)]
    pub ico_ilim: B5,
    #[skip(setters, getters)]
    reserved: B3,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerStatus1 {
    #[skip(setters)]
    pub chrg_stat: B3,
    #[skip(setters)]
    pub wd_stat: B1,
    #[skip(setters)]
    pub treg_stat: B1,
    #[skip(setters)]
    pub vindpm_stat: B1,
    #[skip(setters)]
    pub iindpm_stat: B1,
    #[skip(setters, getters)]
    reserved: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerStatus2 {
    #[skip(setters, getters)]
    reserved_1: B1,
    #[skip(setters)]
    pub ico_stat: B2,
    #[skip(setters, getters)]
    reserved_2: B1,
    #[skip(setters)]
    pub vbus_stat: B3,
    #[skip(setters)]
    pub pg_stat: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct NTCStatus {
    #[skip(setters)]
    pub ts_stat: B3,
    #[skip(setters, getters)]
    reserved: B5,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct FaultStatus {
    #[skip(setters, getters)]
    reserved_1: B1,
    #[skip(setters, getters)]
    reserved_2: B3,
    #[skip(setters)]
    pub tmr_stat: B1,
    #[skip(setters, getters)]
    reserved_3: B1,
    #[skip(setters)]
    pub tshut_stat: B1,
    #[skip(setters)]
    pub vbus_ovp_stat: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerFlag1 {
    #[skip(setters)]
    pub chrg_flag: B1,
    #[skip(setters, getters)]
    reserved_1: B2,
    #[skip(setters)]
    pub wd_flag: B1,
    #[skip(setters)]
    pub treg_flag: B1,
    #[skip(setters)]
    pub vindpm_flag: B1,
    #[skip(setters)]
    pub iindpm_flag: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerFlag2 {
    #[skip(setters, getters)]
    reserved_1: B1,
    #[skip(setters)]
    pub ico_flag: B1,
    #[skip(setters)]
    pub ts_flag: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    #[skip(setters)]
    pub vbus_flag: B1,
    #[skip(setters, getters)]
    reserved_3: B2,
    #[skip(setters)]
    pub pg_flag: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct FaultFlag {
    #[skip(setters, getters)]
    reserved_1: B4,
    #[skip(setters)]
    pub tmr_flag: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    #[skip(setters)]
    pub tshut_flag: B1,
    #[skip(setters)]
    pub vbus_ovp_flag: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerMask1 {
    pub chrg_mask: B1,
    #[skip(setters, getters)]
    reserved: B2,
    pub wd_mask: B1,
    pub treg_mask: B1,
    pub vinpdm_mask: B1,
    pub iindpm_mask: B1,
    pub adc_done_mask: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerMask2 {
    #[skip(setters, getters)]
    reserved_1: B1,
    #[skip(setters)]
    pub ico_mask: B1,
    #[skip(setters)]
    pub ts_mask: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    #[skip(setters)]
    pub vbus_mask: B1,
    #[skip(setters, getters)]
    reserved_3: B2,
    #[skip(setters)]
    pub pg_mask: B1,
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

        Ok(ChargeCurrentLimit::from_bytes([byte]))
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
        buf[0] = current_limit.bytes[0];
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

        Ok(InputVoltageLimit::from_bytes([byte]))
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
        buf[0] = input_volt_limit.bytes[0];
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
        Ok(InputCurrentLimit::from_bytes([byte]))
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
        buf[0] = current_limit.into_bytes()[0];
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
    pub async fn read_precharge_and_termination_current_limit(
        &mut self,
    ) -> Result<PrechargeAndTerminationCurrentLimit, BQ25887Error<I2C::Error>> {
        let reg = self.device.prechg_termination_ctrl().read_async().await?;

        let iprechg: u8 = reg.iprechg()?.into();
        let iterm: u8 = reg.iprechg()?.into();
        let byte = ((iprechg << 4) & 0b1111_0000) | (iterm & 0b0000_1111);
        Ok(PrechargeAndTerminationCurrentLimit::from_bytes([byte]))
    }

    /// ### Breif
    /// Writes Precharge and Termination Current Limit Register,
    /// (Address = 0x04) [reset = 0x22]
    /// BQ255887 p.37
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_precharge_and_termination_current_limit(
        &mut self,
        current_limit: PrechargeAndTerminationCurrentLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = current_limit.bytes[0];
        self.device
            .interface()
            .write_register(PRECHARGE_AND_TERMINATION_CURRENT_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }

    /// ### Breif
    /// Reads Charger Control 1 Register,
    /// (Address = 0x05) (reset = 0x9D)
    /// BQ255887 p.38
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_control_1(&mut self) -> Result<ChargerControl1, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_ctrl_1().read_async().await?;

        let en_term = u8::from(reg.en_term()) << 7;
        let stat_dis = u8::from(reg.stat_dis()) << 6;
        let watchdog: u8 = reg.watchdog().into();
        let en_timer = u8::from(reg.en_timer()) << 3;
        let chg_timer: u8 = reg.chg_timer().into();
        let tmr2x_en = u8::from(reg.tmr_2_x_en());

        let byte = en_term
            | stat_dis
            | ((watchdog << 4) & 0b0011_0000)
            | en_timer
            | ((chg_timer << 1) & 0b0000_0110)
            | tmr2x_en;
        Ok(ChargerControl1::from_bytes([byte]))
    }

    /// ### Breif
    /// Writes Charger Control 1 Register,
    /// (Address = 0x05) (reset = 0x9D)
    /// BQ255887 p.38
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_control_1(&mut self, control: ChargerControl1) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = control.bytes[0];
        self.device
            .interface()
            .write_register(CHARGER_CONTROL_1_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }

    /// ### Breif
    /// Reads Charger Control 2 Register,
    /// (Address = 0x06) (reset = 0x7D)
    /// BQ255887 p.39
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_control_2(&mut self) -> Result<ChargerControl2, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_ctrl_2().read_async().await?;

        let auto_indet_en = u8::from(reg.auto_indet_en()) << 6;
        let treg: u8 = reg.treg().into();
        let en_chg = u8::from(reg.en_chg()) << 3;
        let celllowv = u8::from(reg.celllowv()) << 2;
        let vrechg: u8 = reg.vcell_rechg().into();

        let byte = auto_indet_en | ((treg << 4) & 0b0011_0000) | en_chg | celllowv | (vrechg & 0b0000_0011);
        Ok(ChargerControl2::from_bytes([byte]))
    }

    /// ### Breif
    /// Write Charger Control 2 Register,
    /// (Address = 0x06) (reset = 0x7D)
    /// BQ255887 p.39
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_control_2(&mut self, control: ChargerControl2) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = control.bytes[0];
        self.device
            .interface()
            .write_register(CHARGER_CONTROL_2_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }

    /// ### Breif
    /// Reads Charger Control 3 Register,
    /// (Address = 0x07) (reset = 0x00)
    /// BQ255887 p.40
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_control_3(&mut self) -> Result<ChargerControl3, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_ctrl_3().read_async().await?;

        let pfm_dis = u8::from(reg.pfm_dis()) << 7;
        let wd_rst = u8::from(reg.wd_rst()) << 6;
        let topoff_timer: u8 = reg.topoff_timer().into();

        let byte = pfm_dis | wd_rst | ((topoff_timer << 4) & 0b0011_0000);
        Ok(ChargerControl3::from_bytes([byte]))
    }

    /// ### Breif
    /// Writes Charger Control 3 Register,
    /// (Address = 0x07) (reset = 0x00)
    /// BQ255887 p.40
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_control_3(&mut self, control: ChargerControl3) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = control.bytes[0];
        self.device
            .interface()
            .write_register(CHARGER_CONTROL_3_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }

    /// ### Breif
    /// Reads Charger Control 4 Register,
    /// (Address = 0x08) (reset = 0x0D)
    /// BQ255887 p.41
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_control_4(&mut self) -> Result<ChargerControl4, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_ctrl_4().read_async().await?;

        let vset: u8 = reg.jeita_vset().into();
        let iseth = u8::from(reg.jeita_isetc()) << 2;
        let isetcurr: u8 = reg.jeita_isetc().into();

        let byte = ((vset << 3) & 0b0001_1000) | iseth | (isetcurr & 0b0000_0011);
        Ok(ChargerControl4::from_bytes([byte]))
    }

    /// ### Breif
    /// Writes Charger Control 4 Register,
    /// (Address = 0x08) (reset = 0x0D)
    /// BQ255887 p.41
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_control_4(&mut self, control: ChargerControl4) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = control.bytes[0];
        self.device
            .interface()
            .write_register(CHARGER_CONTROL_4_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }

    /// ### Breif
    /// Reads ICO Current Limit in Use Register,
    /// (Address = 0x0A) (reset = 0x??)
    /// BQ255887 p.43
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ico_current_limit_in_use(&mut self) -> Result<IcoCurrentLimitInUse, BQ25887Error<I2C::Error>> {
        let reg = self.device.ico_current_limit().read_async().await?;

        Ok(IcoCurrentLimitInUse::from_bytes([reg.ico_ilim()]))
    }

    /// ### Breif
    /// Reads Charger Status 1 Register,
    /// (Address = 0x0B) [reset = 0x??]
    /// BQ255887 p.44
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_status_1(&mut self) -> Result<ChargerStatus1, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_status_1().read_async().await?;

        let iindpm_stat = u8::from(reg.iindpm_stat()) << 6;
        let vindpm_stat = u8::from(reg.vindpm_stat()) << 5;
        let treg_stat = u8::from(reg.treg_stat()) << 4;
        let wd_stat = u8::from(reg.wd_stat()) << 3;
        let chrg_stat: u8 = reg.chrg_stat().into();

        let byte = iindpm_stat | vindpm_stat | treg_stat | wd_stat | (chrg_stat & 0b0000_0111);

        Ok(ChargerStatus1::from_bytes([byte]))
    }

    /// ### Breif
    /// Reads Charger Status 2 Register,
    /// (Address = 0x0C) [reset = 0x??]
    /// BQ255887 p.45
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_status_2(&mut self) -> Result<ChargerStatus2, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_status_2().read_async().await?;

        let pg_stat = u8::from(reg.pg_stat()) << 7;
        let vbus_stat: u8 = reg.vbus_stat().into();
        let ico_stat: u8 = reg.ico_stat().into();

        let byte = pg_stat | ((vbus_stat << 4) & 0b0111_0000) | ((ico_stat << 1) & 0b0000_0110);

        Ok(ChargerStatus2::from_bytes([byte]))
    }

    /// ### Breif
    /// Reads NTC Status Register,
    /// (Address = 0x0D) [reset = 0x??]
    /// BQ255887 p.46
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ntc_status(&mut self) -> Result<NTCStatus, BQ25887Error<I2C::Error>> {
        let reg = self.device.ntc_status().read_async().await?;

        let ts_stat: u8 = reg.ts_stat()?.into();
        Ok(NTCStatus::from_bytes([ts_stat]))
    }

    /// ### Breif
    /// Reads FAULT Status Register,
    /// (Address = 0x0E) [reset = 0x??]
    /// BQ255887 p.47
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_fault_status(&mut self) -> Result<FaultStatus, BQ25887Error<I2C::Error>> {
        let reg = self.device.fault_status().read_async().await?;

        let vbus_ovp_stat = u8::from(reg.vbus_ovp_stat()) << 7;
        let tshut_stat = u8::from(reg.tshut_stat()) << 6;
        let tmr_stat = u8::from(reg.tmr_stat()) << 4;

        let byte = vbus_ovp_stat | tshut_stat | tmr_stat;
        Ok(FaultStatus::from_bytes([byte]))
    }

    /// ### Breif
    /// Reads  Charger Flag 1 Register,
    /// (Address = 0x0F) [reset = 0x??]
    /// BQ255887 p.48
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_flag_1(&mut self) -> Result<ChargerFlag1, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_flag_1().read_async().await?;

        let iindpm = u8::from(reg.iindpm_flag()) << 6;
        let vindpm = u8::from(reg.vindpm_flag()) << 5;
        let treg = u8::from(reg.treg_flag()) << 4;
        let wd = u8::from(reg.wd_flag()) << 3;
        let chrg = u8::from(reg.chrg_flag());

        let byte = iindpm | vindpm | treg | wd | chrg;
        Ok(ChargerFlag1::from_bytes([byte]))
    }

    /// ### Breif
    /// Reads  Charger Flag 2 Register,
    /// (Address = 0x10) [reset = 0x??]
    /// BQ255887 p.49
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_flag_2(&mut self) -> Result<ChargerFlag2, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_flag_2().read_async().await?;

        let pg = u8::from(reg.pg_flag()) << 7;
        let vbus = u8::from(reg.vbus_flag()) << 4;
        let ts = u8::from(reg.ts_flag()) << 2;
        let ico = u8::from(reg.ico_flag()) << 1;

        let byte = pg | vbus | ts | ico;
        Ok(ChargerFlag2::from_bytes([byte]))
    }

    /// ### Breif
    /// Reads FAULT Flag Register,
    /// (Address = 0x11) (reset = 0x00)
    /// BQ255887 p.50
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_fault_flag(&mut self) -> Result<FaultFlag, BQ25887Error<I2C::Error>> {
        let reg = self.device.fault_flag().read_async().await?;

        let vbus_ovp = u8::from(reg.vbus_ovp_flag()) << 7;
        let tshut = u8::from(reg.tshut_flag()) << 6;
        let tmr = u8::from(reg.tmr_flag()) << 4;

        let byte = vbus_ovp | tshut | tmr;
        Ok(FaultFlag::from_bytes([byte]))
    }

    /// ### Breif
    /// Reads Charger Mask 1 Register,
    /// (Address = 0x12) (reset = 0x00)
    /// BQ255887 p.51
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_mask_1(&mut self) -> Result<ChargerMask1, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_mask_1().read_async().await?;

        let adc_done = u8::from(reg.adc_done_mask());
        let iindpm = u8::from(reg.iindpm_mask());
        let vindpm = u8::from(reg.vindpm_mask());
        let treg = u8::from(reg.treg_mask());
        let wd = u8::from(reg.wd_mask());
        let chrg = u8::from(reg.chrg_mask());

        Ok(ChargerMask1::new()
            .with_adc_done_mask(adc_done)
            .with_iindpm_mask(iindpm)
            .with_vinpdm_mask(vindpm)
            .with_treg_mask(treg)
            .with_wd_mask(wd)
            .with_chrg_mask(chrg))
    }

    /// ### Breif
    /// Writes Charger Mask 1 Register,
    /// (Address = 0x12) (reset = 0x00)
    /// BQ255887 p.51
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_mask_1(&mut self, mask: ChargerMask1) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = mask.bytes[0];
        self.device
            .interface()
            .write_register(CHARGER_MASK_1_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }

    /// ### Breif
    /// Reads Charger Mask 2 Register,
    /// (Address = 0x13) (reset = 0x00)
    /// BQ255887 p.52
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_mask_2(&mut self) -> Result<ChargerMask2, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_mask_2().read_async().await?;

        let pg = u8::from(reg.pg_mask()) << 7;
        let vbus = u8::from(reg.vbus_mask()) << 4;
        let ts = u8::from(reg.ts_mask()) << 2;
        let ico = u8::from(reg.ico_mask()) << 1;

        let byte = pg | vbus | ts | ico;
        Ok(ChargerMask2::from_bytes([byte]))
    }

    /// ### Breif
    /// Writes Charger Mask 2 Register,
    /// (Address = 0x13) (reset = 0x00)
    /// BQ255887 p.52
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_mask_2(&mut self, mask: ChargerMask2) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = mask.bytes[0];
        self.device
            .interface()
            .write_register(CHARGER_MASK_2_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }
}
