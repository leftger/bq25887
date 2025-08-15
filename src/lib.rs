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
const FAULT_MASK_ADDR: u8 = 0x14;

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
    pub ico_ilim: B5,
    #[skip(setters, getters)]
    reserved: B3,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerStatus1 {
    pub chrg_stat: B3,
    pub wd_stat: B1,
    pub treg_stat: B1,
    pub vindpm_stat: B1,
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
    pub ico_stat: B2,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub vbus_stat: B3,
    pub pg_stat: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct NTCStatus {
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
    pub tmr_stat: B1,
    #[skip(setters, getters)]
    reserved_3: B1,
    pub tshut_stat: B1,
    pub vbus_ovp_stat: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerFlag1 {
    pub chrg_flag: B1,
    #[skip(setters, getters)]
    reserved_1: B2,
    pub wd_flag: B1,
    pub treg_flag: B1,
    pub vindpm_flag: B1,
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
    pub ico_flag: B1,
    pub ts_flag: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub vbus_flag: B1,
    #[skip(setters, getters)]
    reserved_3: B2,
    pub pg_flag: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct FaultFlag {
    #[skip(setters, getters)]
    reserved_1: B4,
    pub tmr_flag: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub tshut_flag: B1,
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
    pub ico_mask: B1,
    pub ts_mask: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub vbus_mask: B1,
    #[skip(setters, getters)]
    reserved_3: B2,
    pub pg_mask: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct FaultMask{
    #[skip(setters, getters)]
    reserved_1: B3,
    pub sns_short_mask: B1,
    pub tmr_mask: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub tshut_mask: B1,
    pub vbus_ovp_mask: B1,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AdcControl{
    #[skip(setters, getters)]
    reserved: B4,
    pub adc_sample: B2,
    pub adc_rate: B1,
    pub adc_en: B1,
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

        let en_hiz = u8::from(reg.en_hiz());
        let en_ilim = u8::from(reg.en_ilim());
        let ichg = u8::from(reg.ichg()?);

        Ok(ChargeCurrentLimit::new()
            .with_en_hiz(en_hiz)
            .with_en_ilim(en_ilim)
            .with_ichg(ichg))
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

        let en_vindpm_rst = u8::from(reg.en_vindpm_rst());
        let en_bat_dischg = u8::from(reg.en_bat_dischg());
        let pfm_ooa_dis = u8::from(reg.pfm_ooa_dis());
        let vindpm = u8::from(reg.vindpm()?);

        Ok(InputVoltageLimit::new()
            .with_en_vindpm_rst(en_vindpm_rst)
            .with_en_bat_dischg(en_bat_dischg)
            .with_pfm_ooa_dis(pfm_ooa_dis)
            .with_vindpm(vindpm))
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

        let force_ico = u8::from(reg.force_ico());
        let force_indet = u8::from(reg.force_indet());
        let en_ico = u8::from(reg.en_ico());
        let iindpm = u8::from(reg.iindpm()?);

        Ok(InputCurrentLimit::new()
            .with_force_ico(force_ico)
            .with_force_indet(force_indet)
            .with_en_ico(en_ico)
            .with_iindpm(iindpm))
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
        buf[0] = current_limit.bytes[0];
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

        let iprechg = u8::from(reg.iprechg()?);
        let iterm = u8::from(reg.iprechg()?);

        Ok(PrechargeAndTerminationCurrentLimit::new()
            .with_iprechg(iprechg)
            .with_iterm(iterm))
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

        let en_term = u8::from(reg.en_term());
        let stat_dis = u8::from(reg.stat_dis());
        let watchdog = u8::from(reg.watchdog());
        let en_timer = u8::from(reg.en_timer());
        let chg_timer= u8::from(reg.chg_timer());
        let tmr2x_en = u8::from(reg.tmr_2_x_en());

        Ok(ChargerControl1::new()
            .with_en_term(en_term)
            .with_stat_dis(stat_dis)
            .with_watchdog(watchdog)
            .with_en_timer(en_timer)
            .with_chg_timer(chg_timer)
            .with_tmr2x_en(tmr2x_en))
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

        let auto_indet_en = u8::from(reg.auto_indet_en());
        let treg = u8::from(reg.treg());
        let en_chg = u8::from(reg.en_chg());
        let celllowv = u8::from(reg.celllowv());
        let vrechg = u8::from(reg.vcell_rechg());

        Ok(ChargerControl2::new()
            .with_auto_indet_en(auto_indet_en)
            .with_treg(treg)
            .with_en_chg(en_chg)
            .with_celllowv(celllowv)
            .with_vrechg(vrechg))
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

        let pfm_dis = u8::from(reg.pfm_dis());
        let wd_rst = u8::from(reg.wd_rst());
        let topoff_timer = u8::from(reg.topoff_timer());

        Ok(ChargerControl3::new()
            .with_pfm_dis(pfm_dis)
            .with_wd_rst(wd_rst)
            .with_topoff_timer(topoff_timer))
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

        let vset = u8::from(reg.jeita_vset());
        let iseth = u8::from(reg.jeita_iseth());
        let isetcurr = u8::from(reg.jeita_isetc());

        Ok(ChargerControl4::new()
            .with_jeita_vset(vset)
            .with_jeita_iseth(iseth)
            .with_jeita_isetc(isetcurr))
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

        Ok(IcoCurrentLimitInUse::new().with_ico_ilim(reg.ico_ilim()))
    }

    /// ### Breif
    /// Reads Charger Status 1 Register,
    /// (Address = 0x0B) [reset = 0x??]
    /// BQ255887 p.44
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_status_1(&mut self) -> Result<ChargerStatus1, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_status_1().read_async().await?;

        let iindpm_stat = u8::from(reg.iindpm_stat());
        let vindpm_stat = u8::from(reg.vindpm_stat());
        let treg_stat = u8::from(reg.treg_stat());
        let wd_stat = u8::from(reg.wd_stat());
        let chrg_stat = u8::from(reg.chrg_stat());

        Ok(ChargerStatus1::new()
            .with_iindpm_stat(iindpm_stat)
            .with_vindpm_stat(vindpm_stat)
            .with_treg_stat(treg_stat)
            .with_wd_stat(wd_stat)
            .with_chrg_stat(chrg_stat))
    }

    /// ### Breif
    /// Reads Charger Status 2 Register,
    /// (Address = 0x0C) [reset = 0x??]
    /// BQ255887 p.45
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_status_2(&mut self) -> Result<ChargerStatus2, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_status_2().read_async().await?;

        let pg_stat = u8::from(reg.pg_stat());
        let vbus_stat = u8::from(reg.vbus_stat());
        let ico_stat = u8::from(reg.ico_stat());

        Ok(ChargerStatus2::new()
            .with_pg_stat(pg_stat)
            .with_vbus_stat(vbus_stat)
            .with_ico_stat(ico_stat))
    }

    /// ### Breif
    /// Reads NTC Status Register,
    /// (Address = 0x0D) [reset = 0x??]
    /// BQ255887 p.46
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ntc_status(&mut self) -> Result<NTCStatus, BQ25887Error<I2C::Error>> {
        let reg = self.device.ntc_status().read_async().await?;

        let ts_stat = u8::from(reg.ts_stat()?);

        Ok(NTCStatus::new().with_ts_stat(ts_stat))
    }

    /// ### Breif
    /// Reads FAULT Status Register,
    /// (Address = 0x0E) [reset = 0x??]
    /// BQ255887 p.47
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_fault_status(&mut self) -> Result<FaultStatus, BQ25887Error<I2C::Error>> {
        let reg = self.device.fault_status().read_async().await?;

        let vbus_ovp_stat = u8::from(reg.vbus_ovp_stat());
        let tshut_stat = u8::from(reg.tshut_stat());
        let tmr_stat = u8::from(reg.tmr_stat());

        Ok(FaultStatus::new()
            .with_vbus_ovp_stat(vbus_ovp_stat)
            .with_tshut_stat(tshut_stat)
            .with_tmr_stat(tmr_stat))
    }

    /// ### Breif
    /// Reads  Charger Flag 1 Register,
    /// (Address = 0x0F) [reset = 0x??]
    /// BQ255887 p.48
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_flag_1(&mut self) -> Result<ChargerFlag1, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_flag_1().read_async().await?;

        let iindpm = u8::from(reg.iindpm_flag());
        let vindpm = u8::from(reg.vindpm_flag());
        let treg = u8::from(reg.treg_flag());
        let wd = u8::from(reg.wd_flag());
        let chrg = u8::from(reg.chrg_flag());

        Ok(ChargerFlag1::new()
            .with_iindpm_flag(iindpm)
            .with_vindpm_flag(vindpm)
            .with_treg_flag(treg)
            .with_wd_flag(wd)
            .with_chrg_flag(chrg))
    }

    /// ### Breif
    /// Reads  Charger Flag 2 Register,
    /// (Address = 0x10) [reset = 0x??]
    /// BQ255887 p.49
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_flag_2(&mut self) -> Result<ChargerFlag2, BQ25887Error<I2C::Error>> {
        let reg = self.device.charger_flag_2().read_async().await?;

        let pg = u8::from(reg.pg_flag());
        let vbus = u8::from(reg.vbus_flag());
        let ts = u8::from(reg.ts_flag());
        let ico = u8::from(reg.ico_flag());

        Ok(ChargerFlag2::new()
            .with_pg_flag(pg)
            .with_vbus_flag(vbus)
            .with_ts_flag(ts)
            .with_ico_flag(ico))
    }

    /// ### Breif
    /// Reads FAULT Flag Register,
    /// (Address = 0x11) (reset = 0x00)
    /// BQ255887 p.50
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_fault_flag(&mut self) -> Result<FaultFlag, BQ25887Error<I2C::Error>> {
        let reg = self.device.fault_flag().read_async().await?;

        let vbus_ovp = u8::from(reg.vbus_ovp_flag());
        let tshut = u8::from(reg.tshut_flag());
        let tmr = u8::from(reg.tmr_flag());

        Ok(FaultFlag::new()
            .with_vbus_ovp_flag(vbus_ovp)
            .with_tshut_flag(tshut)
            .with_tmr_flag(tmr))
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

        let pg = u8::from(reg.pg_mask());
        let vbus = u8::from(reg.vbus_mask());
        let ts = u8::from(reg.ts_mask());
        let ico = u8::from(reg.ico_mask());

        Ok(ChargerMask2::new()
            .with_pg_mask(pg)
            .with_vbus_mask(vbus)
            .with_ts_mask(ts)
            .with_ico_mask(ico))
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

    /// ### Breif
    /// Reads FAULT Mask Register,
    /// (Address = 0x14) (reset = 0x00)
    /// BQ255887 p.53
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_fault_mask(&mut self) -> Result<FaultMask, BQ25887Error<I2C::Error>>{
        let reg = self.device.fault_mask().read_async().await?;

        let vbus = u8::from(reg.vbus_ovp_mask());
        let tshut = u8::from(reg.tshut_mask());
        let tmr = u8::from(reg.tmr_mask());
        let sns_short = u8::from(reg.sns_short_mask());

        Ok(FaultMask::new().with_vbus_ovp_mask(vbus).with_tshut_mask(tshut).with_tmr_mask(tmr).with_sns_short_mask(sns_short))
    }

    /// ### Breif
    /// Writes FAULT Mask Register,
    /// (Address = 0x14) (reset = 0x00)
    /// BQ255887 p.53
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_fault_mask(&mut self, mask: FaultMask) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut buf = [0u8; LARGEST_REG_SIZE_BYTES];
        buf[0] = mask.bytes[0];
        self.device
            .interface()
            .write_register(FAULT_MASK_ADDR, LARGEST_REG_SIZE_BITS, &buf)
            .await?;
        Ok(())
    }

    /// ### Breif
    /// Reads ADC Control Register,
    /// (Address = 0x15) (reset = 0x00)
    /// BQ255887 p.54
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_adc_control(&mut self) -> Result<AdcControl, BQ25887Error<I2C::Error>>{
        let reg = self.device.adc_control().read_async().await?;

        let adc_en = u8::from(reg.adc_en());
        let adc_rate = u8::from(reg.adc_rate());
        let adc_sample = u8::from(reg.adc_sample());

        Ok(AdcControl::new().with_adc_en(adc_en).with_adc_rate(adc_rate).with_adc_sample(adc_sample))
    }

}
