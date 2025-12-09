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
use embedded_hal_async::i2c::I2c as I2cTrait;

const BQ_ADDR: u8 = 0x6A;
const LARGEST_REG_SIZE_BYTES: usize = 0x01;

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
    pub async fn read_voltage_regulation_limit(
        &mut self,
    ) -> Result<crate::field_sets::CellVoltageLimit, BQ25887Error<I2C::Error>> {
        self.device.cell_voltage_limit().read_async().await
    }

    /// ### Breif
    /// Writes the given `CellVoltageLimit` to the Cell Voltage Regulation Limit Register,
    /// (Address = 0x00) (reset = 0xA0),
    /// BQ25887 p.33
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_voltage_regulation_limit(
        &mut self,
        volt_limit: crate::field_sets::CellVoltageLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .cell_voltage_limit()
            .write_async(|reg| *reg = volt_limit)
            .await
    }

    /// ### Breif
    /// Reads Charger Current Limit Register,
    /// (Address = 0x01) (reset = 0x5E),
    /// BQ25887 p.34
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_charge_current_limit(
        &mut self,
    ) -> Result<crate::field_sets::ChargeCurrentLimit, BQ25887Error<I2C::Error>> {
        self.device.charge_current_limit().read_async().await
    }

    /// ### Breif
    /// Writes to the Charger Current Limit Register,
    /// (Address = 0x01) (reset = 0x5E),
    /// BQ25887 p.34
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charge_current_limit(
        &mut self,
        current_limit: crate::field_sets::ChargeCurrentLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .charge_current_limit()
            .write_async(|reg| *reg = current_limit)
            .await
    }

    /// ### Breif
    /// Reads Input Voltage Limit Register,
    /// (Address = 02h) (reset = 84h),
    /// BQ25887 p.35
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_input_voltage_limit(
        &mut self,
    ) -> Result<crate::field_sets::InputVoltageLimit, BQ25887Error<I2C::Error>> {
        self.device.input_voltage_limit().read_async().await
    }

    /// ### Breif
    /// Writes Input Voltage Limit Register,
    /// (Address = 02h) (reset = 84h),
    /// BQ25887 p.35
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_input_voltage_limit(
        &mut self,
        input_volt_limit: crate::field_sets::InputVoltageLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .input_voltage_limit()
            .write_async(|reg| *reg = input_volt_limit)
            .await
    }

    /// ### Breif
    /// Reads Input Current Limit Register,
    /// (Address = 0x03) (reset = 0x39 )
    /// BQ255887 p.36
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_input_current_limit(
        &mut self,
    ) -> Result<crate::field_sets::InputCurrentLimit, BQ25887Error<I2C::Error>> {
        self.device.input_current_limit().read_async().await
    }

    /// ### Breif
    /// Writes Input Current Limit Register,
    /// (Address = 0x03) (reset = 0x39 )
    /// BQ255887 p.36
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_input_current_limit(
        &mut self,
        current_limit: crate::field_sets::InputCurrentLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .input_current_limit()
            .write_async(|reg| *reg = current_limit)
            .await
    }

    /// ### Breif
    /// Reads Precharge and Termination Current Limit Register,
    /// (Address = 0x04) [reset = 0x22]
    /// BQ255887 p.37
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_precharge_and_termination_current_limit(
        &mut self,
    ) -> Result<crate::field_sets::PrechgTerminationCtrl, BQ25887Error<I2C::Error>> {
        self.device.prechg_termination_ctrl().read_async().await
    }

    /// ### Breif
    /// Writes Precharge and Termination Current Limit Register,
    /// (Address = 0x04) [reset = 0x22]
    /// BQ255887 p.37
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_precharge_and_termination_current_limit(
        &mut self,
        current_limit: crate::field_sets::PrechgTerminationCtrl,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .prechg_termination_ctrl()
            .write_async(|reg| *reg = current_limit)
            .await
    }

    /// ### Breif
    /// Reads Charger Control 1 Register,
    /// (Address = 0x05) (reset = 0x9D)
    /// BQ255887 p.38
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_control_1(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl1, BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_1().read_async().await
    }

    /// ### Breif
    /// Writes Charger Control 1 Register,
    /// (Address = 0x05) (reset = 0x9D)
    /// BQ255887 p.38
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_control_1(
        &mut self,
        control: crate::field_sets::ChargerCtrl1,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_1().write_async(|reg| *reg = control).await
    }

    /// ### Breif
    /// Reads Charger Control 2 Register,
    /// (Address = 0x06) (reset = 0x7D)
    /// BQ255887 p.39
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_control_2(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl2, BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_2().read_async().await
    }

    /// ### Breif
    /// Write Charger Control 2 Register,
    /// (Address = 0x06) (reset = 0x7D)
    /// BQ255887 p.39
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_control_2(
        &mut self,
        control: crate::field_sets::ChargerCtrl2,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_2().write_async(|reg| *reg = control).await
    }

    /// ### Breif
    /// Reads Charger Control 3 Register,
    /// (Address = 0x07) (reset = 0x00)
    /// BQ255887 p.40
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_control_3(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl3, BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_3().read_async().await
    }

    /// ### Breif
    /// Writes Charger Control 3 Register,
    /// (Address = 0x07) (reset = 0x00)
    /// BQ255887 p.40
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_control_3(
        &mut self,
        control: crate::field_sets::ChargerCtrl3,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_3().write_async(|reg| *reg = control).await
    }

    /// ### Breif
    /// Reads Charger Control 4 Register,
    /// (Address = 0x08) (reset = 0x0D)
    /// BQ255887 p.41
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_control_4(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl4, BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_4().read_async().await
    }

    /// ### Breif
    /// Writes Charger Control 4 Register,
    /// (Address = 0x08) (reset = 0x0D)
    /// BQ255887 p.41
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_control_4(
        &mut self,
        control: crate::field_sets::ChargerCtrl4,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_4().write_async(|reg| *reg = control).await
    }

    /// ### Breif
    /// Reads ICO Current Limit in Use Register,
    /// (Address = 0x0A) (reset = 0x??)
    /// BQ255887 p.43
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ico_current_limit_in_use(
        &mut self,
    ) -> Result<crate::field_sets::IcoCurrentLimit, BQ25887Error<I2C::Error>> {
        self.device.ico_current_limit().read_async().await
    }

    /// ### Breif
    /// Reads Charger Status 1 Register,
    /// (Address = 0x0B) [reset = 0x??]
    /// BQ255887 p.44
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_status_1(
        &mut self,
    ) -> Result<crate::field_sets::ChargerStatus1, BQ25887Error<I2C::Error>> {
        self.device.charger_status_1().read_async().await
    }

    /// ### Breif
    /// Reads Charger Status 2 Register,
    /// (Address = 0x0C) [reset = 0x??]
    /// BQ255887 p.45
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_status_2(
        &mut self,
    ) -> Result<crate::field_sets::ChargerStatus2, BQ25887Error<I2C::Error>> {
        self.device.charger_status_2().read_async().await
    }

    /// ### Breif
    /// Reads NTC Status Register,
    /// (Address = 0x0D) [reset = 0x??]
    /// BQ255887 p.46
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ntc_status(&mut self) -> Result<crate::field_sets::NtcStatus, BQ25887Error<I2C::Error>> {
        self.device.ntc_status().read_async().await
    }

    /// ### Breif
    /// Reads FAULT Status Register,
    /// (Address = 0x0E) [reset = 0x??]
    /// BQ255887 p.47
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_fault_status(&mut self) -> Result<crate::field_sets::FaultStatus, BQ25887Error<I2C::Error>> {
        self.device.fault_status().read_async().await
    }

    /// ### Breif
    /// Reads  Charger Flag 1 Register,
    /// (Address = 0x0F) [reset = 0x??]
    /// BQ255887 p.48
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_flag_1(&mut self) -> Result<crate::field_sets::ChargerFlag1, BQ25887Error<I2C::Error>> {
        self.device.charger_flag_1().read_async().await
    }

    /// ### Breif
    /// Reads  Charger Flag 2 Register,
    /// (Address = 0x10) [reset = 0x??]
    /// BQ255887 p.49
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_flag_2(&mut self) -> Result<crate::field_sets::ChargerFlag2, BQ25887Error<I2C::Error>> {
        self.device.charger_flag_2().read_async().await
    }

    /// ### Breif
    /// Reads FAULT Flag Register,
    /// (Address = 0x11) (reset = 0x00)
    /// BQ255887 p.50
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_fault_flag(&mut self) -> Result<crate::field_sets::FaultFlag, BQ25887Error<I2C::Error>> {
        self.device.fault_flag().read_async().await
    }

    /// ### Breif
    /// Reads Charger Mask 1 Register,
    /// (Address = 0x12) (reset = 0x00)
    /// BQ255887 p.51
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_mask_1(&mut self) -> Result<crate::field_sets::ChargerMask1, BQ25887Error<I2C::Error>> {
        self.device.charger_mask_1().read_async().await
    }

    /// ### Breif
    /// Writes Charger Mask 1 Register,
    /// (Address = 0x12) (reset = 0x00)
    /// BQ255887 p.51
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_mask_1(
        &mut self,
        mask: crate::field_sets::ChargerMask1,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_mask_1().write_async(|reg| *reg = mask).await
    }

    /// ### Breif
    /// Reads Charger Mask 2 Register,
    /// (Address = 0x13) (reset = 0x00)
    /// BQ255887 p.52
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_charger_mask_2(&mut self) -> Result<crate::field_sets::ChargerMask2, BQ25887Error<I2C::Error>> {
        self.device.charger_mask_2().read_async().await
    }

    /// ### Breif
    /// Writes Charger Mask 2 Register,
    /// (Address = 0x13) (reset = 0x00)
    /// BQ255887 p.52
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_charger_mask_2(
        &mut self,
        mask: crate::field_sets::ChargerMask2,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_mask_2().write_async(|reg| *reg = mask).await
    }

    /// ### Breif
    /// Reads FAULT Mask Register,
    /// (Address = 0x14) (reset = 0x00)
    /// BQ255887 p.53
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_fault_mask(&mut self) -> Result<crate::field_sets::FaultMask, BQ25887Error<I2C::Error>> {
        self.device.fault_mask().read_async().await
    }

    /// ### Breif
    /// Writes FAULT Mask Register,
    /// (Address = 0x14) (reset = 0x00)
    /// BQ255887 p.53
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_fault_mask(
        &mut self,
        mask: crate::field_sets::FaultMask,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.fault_mask().write_async(|reg| *reg = mask).await
    }

    /// ### Breif
    /// Reads ADC Control Register,
    /// (Address = 0x15) (reset = 0x00)
    /// BQ255887 p.54
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_adc_control(&mut self) -> Result<crate::field_sets::AdcControl, BQ25887Error<I2C::Error>> {
        self.device.adc_control().read_async().await
    }

    /// ### Breif
    /// Writes ADC Control Register,
    /// (Address = 0x15) (reset = 0x00)
    /// BQ255887 p.54
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_adc_control(
        &mut self,
        control: crate::field_sets::AdcControl,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.adc_control().write_async(|reg| *reg = control).await
    }

    /// ### Breif
    /// Reads ADC Function Disable Register,
    /// (Address = 0x16) (reset = 0x00)
    /// BQ255887 p.55
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_adc_function_disable(
        &mut self,
    ) -> Result<crate::field_sets::AdcFunctionDisable, BQ25887Error<I2C::Error>> {
        self.device.adc_function_disable().read_async().await
    }

    /// ### Breif
    /// Writes ADC Function Disable Register,
    /// (Address = 0x16) (reset = 0x00)
    /// BQ255887 p.55
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_adc_function_disable(
        &mut self,
        function: crate::field_sets::AdcFunctionDisable,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .adc_function_disable()
            .write_async(|reg| *reg = function)
            .await
    }

    /// ### Breif
    /// Reads IBUS ADC 1 Register,
    /// (Address = 0x17) (reset = 0x00)
    /// BQ255887 p.56
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ibus_adc_1(&mut self) -> Result<crate::field_sets::IbusAdc1, BQ25887Error<I2C::Error>> {
        self.device.ibus_adc_1().read_async().await
    }

    /// ### Breif
    /// Reads IBUS ADC 0 Register,
    /// (Address = 0x18) (reset = 0x00)
    /// BQ255887 p.56
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ibus_adc_0(&mut self) -> Result<crate::field_sets::IbusAdc0, BQ25887Error<I2C::Error>> {
        self.device.ibus_adc_0().read_async().await
    }

    /// ### Breif
    /// Reads ICHG ADC 1 Register,
    /// (Address = 0x19) (reset = 0x00)
    /// BQ255887 p.57
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ichg_adc_1(&mut self) -> Result<crate::field_sets::IchgAdc1, BQ25887Error<I2C::Error>> {
        self.device.ichg_adc_1().read_async().await
    }

    /// ### Breif
    /// Reads ICHG ADC 0 Register,
    /// (Address = 0x1A) (reset = 0x00)
    /// BQ255887 p.57
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ichg_adc_0(&mut self) -> Result<crate::field_sets::IchgAdc0, BQ25887Error<I2C::Error>> {
        self.device.ichg_adc_0().read_async().await
    }

    /// ### Breif
    /// Reads VBUS ADC 1 Register,
    /// (Address = 0x1B) (reset = 0x00)
    /// BQ255887 p.58
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vbus_adc_1(&mut self) -> Result<crate::field_sets::VbusAdc1, BQ25887Error<I2C::Error>> {
        self.device.vbus_adc_1().read_async().await
    }

    /// ### Breif
    /// Reads VBUS ADC 0 Register,
    /// (Address = 0x1C) (reset = 0x00)
    /// BQ255887 p.58
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vbus_adc_0(&mut self) -> Result<crate::field_sets::VbusAdc0, BQ25887Error<I2C::Error>> {
        self.device.vbus_adc_0().read_async().await
    }

    /// ### Breif
    /// Reads VBAT ADC 1 Register,
    /// (Address = 0x1D) (reset = 0x00)
    /// BQ255887 p.59
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vbat_adc_1(&mut self) -> Result<crate::field_sets::VbatAdc1, BQ25887Error<I2C::Error>> {
        self.device.vbat_adc_1().read_async().await
    }

    /// ### Breif
    /// Reads VBAT ADC 0 Register,
    /// (Address = 0x1E) (reset = 0x00)
    /// BQ255887 p.59
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vbat_adc_0(&mut self) -> Result<crate::field_sets::VbatAdc0, BQ25887Error<I2C::Error>> {
        self.device.vbat_adc_0().read_async().await
    }

    /// ### Breif
    /// Reads VCELLTOP ADC 1 Register,
    /// (Address = 0x1F) (reset = 0x00)
    /// BQ255887 p.60
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vcell_top_adc_1(&mut self) -> Result<crate::field_sets::VcelltopAdc1, BQ25887Error<I2C::Error>> {
        self.device.vcelltop_adc_1().read_async().await
    }

    /// ### Breif
    /// Reads VCELLTOP ADC 0 Register,
    /// (Address = 0x20) (reset = 0x00)
    /// BQ255887 p.60
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vcell_top_adc_0(&mut self) -> Result<crate::field_sets::VcelltopAdc0, BQ25887Error<I2C::Error>> {
        self.device.vcelltop_adc_0().read_async().await
    }

    /// ### Breif
    /// Reads TS ADC 1 Register,
    /// (Address = 0x21) (reset = 0x00)
    /// BQ255887 p.61
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ts_adc_1(&mut self) -> Result<crate::field_sets::TsAdc1, BQ25887Error<I2C::Error>> {
        self.device.ts_adc_1().read_async().await
    }

    /// ### Breif
    /// Reads TS ADC 0 Register,
    /// (Address = 0x22) (reset = 0x00)
    /// BQ255887 p.61
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_ts_adc_0(&mut self) -> Result<crate::field_sets::TsAdc0, BQ25887Error<I2C::Error>> {
        self.device.ts_adc_0().read_async().await
    }

    /// ### Breif
    /// Reads TDIE ADC 1 Register,
    /// (Address = 0x23) (reset = 0x00)
    /// BQ255887 p.62
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_tdie_adc_1(&mut self) -> Result<crate::field_sets::TdieAdc1, BQ25887Error<I2C::Error>> {
        self.device.tdie_adc_1().read_async().await
    }

    /// ### Breif
    /// Reads TDIE ADC 0 Register,
    /// (Address = 0x24) (reset = 0x00)
    /// BQ255887 p.62
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_tdie_adc_0(&mut self) -> Result<crate::field_sets::TdieAdc0, BQ25887Error<I2C::Error>> {
        self.device.tdie_adc_0().read_async().await
    }

    /// ### Breif
    /// Reads Part Information Register,
    /// (Address = 0x25) (reset = 0x28)
    /// BQ255887 p.63
    /// ### Errors
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed
    pub async fn read_part_information(
        &mut self,
    ) -> Result<crate::field_sets::PartInformation, BQ25887Error<I2C::Error>> {
        self.device.part_information().read_async().await
    }

    /// ### Breif
    /// Writes Part Information Register,
    /// (Address = 0x25) (reset = 0x28)
    /// BQ255887 p.63
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_part_information_reg_rst(
        &mut self,
        info: crate::field_sets::PartInformation,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.part_information().write_async(|reg| *reg = info).await
    }

    /// ### Breif
    /// Reads VCELLBOT ADC 1 Register,
    /// (Address = 0x26) (reset = 0x00)
    /// BQ255887 p.64
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vcellbot_adc_1(&mut self) -> Result<crate::field_sets::VcellbotAdc1, BQ25887Error<I2C::Error>> {
        self.device.vcellbot_adc_1().read_async().await
    }

    /// ### Breif
    /// Reads VCELLBOT ADC 0 Register,
    /// (Address = 0x27) (reset = 0x00)
    /// BQ255887 p.64
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vcellbot_adc_0(&mut self) -> Result<crate::field_sets::VcellbotAdc0, BQ25887Error<I2C::Error>> {
        self.device.vcellbot_adc_0().read_async().await
    }

    /// ### Breif
    /// Reads Cell Balancing Control 1 Register,
    /// (Address = 0x28) (reset = 0x2A)
    /// BQ255887 p.65
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_cell_balancing_control_1(
        &mut self,
    ) -> Result<crate::field_sets::CellBalanceCtrl1, BQ25887Error<I2C::Error>> {
        self.device.cell_balance_ctrl_1().read_async().await
    }

    /// ### Breif
    /// Writes Cell Balancing Control 1 Register,
    /// (Address = 0x28) (reset = 0x2A)
    /// BQ255887 p.65
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_cell_balancing_control_1(
        &mut self,
        control: crate::field_sets::CellBalanceCtrl1,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .cell_balance_ctrl_1()
            .write_async(|reg| *reg = control)
            .await
    }

    /// ### Breif
    /// Reads Cell Balancing Control 2 Register,
    /// (Address = 0x29) (reset = 0xF4)
    /// BQ255887 p.66
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_cell_balancing_control_2(
        &mut self,
    ) -> Result<crate::field_sets::CellBalanceCtrl2, BQ25887Error<I2C::Error>> {
        self.device.cell_balance_ctrl_2().read_async().await
    }

    /// ### Breif
    /// Writes Cell Balancing Control 2 Register,
    /// (Address = 0x29) (reset = 0xF4)
    /// BQ255887 p.66
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_cell_balancing_control_2(
        &mut self,
        control: crate::field_sets::CellBalanceCtrl2,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .cell_balance_ctrl_2()
            .write_async(|reg| *reg = control)
            .await
    }

    /// ### Breif
    /// Reads Cell Balancing Status and Control Register,
    /// (Address = 0x2A) (reset = 0x81)
    /// BQ255887 p.67
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_cell_balancing_status_and_control(
        &mut self,
    ) -> Result<crate::field_sets::CellBalanceStatCtrl, BQ25887Error<I2C::Error>> {
        self.device.cell_balance_stat_ctrl().read_async().await
    }

    /// ### Breif
    /// Writes Cell Balancing Status and Control Register,
    /// (Address = 0x2A) (reset = 0x81)
    /// BQ255887 p.67
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_cell_balancing_status_and_control(
        &mut self,
        control: crate::field_sets::CellBalanceStatCtrl,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .cell_balance_stat_ctrl()
            .write_async(|reg| *reg = control)
            .await
    }

    /// ### Breif
    /// Reads Cell Balancing Flag Register,
    /// (Address = 0x2B) (reset = 0x00)
    /// BQ255887 p.68
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_cell_balancing_flag(
        &mut self,
    ) -> Result<crate::field_sets::CellBalanceFlag, BQ25887Error<I2C::Error>> {
        self.device.cell_balance_flag().read_async().await
    }

    /// ### Breif
    /// Writes Cell Balancing Flag Register,
    /// (Address = 0x2B) (reset = 0x00)
    /// BQ255887 p.68
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_cell_balancing_flag(
        &mut self,
        flag: crate::field_sets::CellBalanceFlag,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.cell_balance_flag().write_async(|reg| *reg = flag).await
    }

    /// ### Breif
    /// Reads Cell Balancing Mask Register,
    /// (Address = 0x2C) (reset = 0x00)
    /// BQ255887 p.68
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_cell_balancing_mask(
        &mut self,
    ) -> Result<crate::field_sets::CellBalanceMask, BQ25887Error<I2C::Error>> {
        self.device.cell_balance_mask().read_async().await
    }

    /// ### Breif
    /// Writes Cell Balancing Mask Register,
    /// (Address = 0x2C) (reset = 0x00)
    /// BQ255887 p.68
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn write_cell_balancing_mask(
        &mut self,
        mask: crate::field_sets::CellBalanceMask,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.cell_balance_mask().write_async(|reg| *reg = mask).await
    }
}
