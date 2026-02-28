//! # BQ25887 battery charger driver
//!
//! `bq25887` is a platform-agnostic Rust driver for the Texas Instruments BQ25887
//! synchronous battery charger. It targets `no_std` environments and provides an
//! async-friendly I²C interface built on the [`embedded-hal-async`] traits.
//!
//! The optional `embassy` feature supplies helpers for constructing shared-bus
//! drivers that integrate smoothly with the Embassy async ecosystem.
//!
//! ## Examples
//!
//! Create a driver with an async I²C peripheral and read the charger status:
//!
//! ```no_run
//! #![cfg_attr(target_os = "none", no_std)]
//! #![cfg_attr(target_os = "none", no_main)]
//!
//! #[cfg(target_os = "none")]
//! use bq25887::Bq25887Driver;
//!
//! #[cfg(target_os = "none")]
//! async fn inspect<I2C>(
//!     i2c: I2C,
//! ) -> core::result::Result<(), bq25887::BQ25887Error<I2C::Error>>
//! where
//!     I2C: embedded_hal_async::i2c::I2c,
//! {
//!     let mut driver = Bq25887Driver::new(i2c);
//!     let status = driver.read_charger_status_1().await?;
//!     let _ = status;
//!     Ok(())
//! }
//!
//! #[cfg(not(target_os = "none"))]
//! fn main() {}
//!
//! # #[cfg(target_os = "none")]
//! # use core::panic::PanicInfo;
//! # #[cfg(target_os = "none")]
//! # #[panic_handler]
//! # fn panic(_info: &PanicInfo) -> ! {
//! #     loop {}
//! # }
//! # #[cfg(target_os = "none")]
//! # #[unsafe(export_name = "_start")]
//! # extern "C" fn start() -> ! {
//! #     loop {}
//! # }
//! ```
//!
//! For a deeper tour of the API and hardware integration tips, see the project
//! README and the device datasheet.
//!
//! [`embedded-hal-async`]: https://docs.rs/embedded-hal-async
//! [datasheet]: https://www.ti.com/lit/ug/slusd89b/slusd89b.pdf
#![cfg_attr(docsrs, feature(doc_cfg))]
#![cfg_attr(not(test), no_std)]
#![warn(missing_docs, rustdoc::broken_intra_doc_links)]
#![doc(html_root_url = "https://docs.rs/bq25887")]

use core::convert::TryFrom;
use core::result::Result;

use embedded_hal_async::i2c::I2c as I2cTrait;

/// Default 7-bit I²C address for the BQ25887 charger.
pub const DEFAULT_I2C_ADDRESS: u8 = 0x6A;
const LARGEST_REG_SIZE_BYTES: usize = 0x01;

#[allow(missing_docs)]
mod generated {
    device_driver::create_device!(
        device_name: Bq25887,
        manifest: "src/bq25887.yaml"
    );
}

/// Type-safe register accessor for the BQ25887 device.
pub use generated::Bq25887;
/// Enumeration of fast charge current limit selections.
pub use generated::Ichg;
/// Enumerated part number identifiers reported by the device.
pub use generated::Pn;
/// ADC conversion rate mode selection.
pub use generated::AdcRate;
/// Generated register field definitions for the charger.
pub use generated::field_sets;
/// Cell recharge threshold offset selection.
pub use generated::VcellRechg;
/// Charge status enumeration.
pub use generated::ChrgStat;
/// Watchdog timer timeout selection.
pub use generated::Watchdog;

#[cfg(feature = "embassy")]
#[cfg_attr(docsrs, doc(cfg(feature = "embassy")))]
pub mod embassy;

/// Error type produced by operations on the BQ25887 driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum BQ25887Error<I2cError> {
    /// I²C transaction failure reported by the underlying bus implementation.
    I2c(I2cError),
    /// Register value conversion failure triggered by invalid raw data.
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
            .write(self.address, &buf[..=data.len()])
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
            .write_read(self.address, &[address], data)
            .await
            .map_err(BQ25887Error::I2c)
    }
}

/// Adapter that bridges the generated register API to an async I²C peripheral.
pub struct DeviceInterface<I2C: I2cTrait> {
    /// Async I²C peripheral used to communicate with the charger.
    pub i2c: I2C,
    address: u8,
}

impl<I2C: I2cTrait> DeviceInterface<I2C> {
    pub(crate) fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }
}

/// Summary of identifying information reported by register 0x25.
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartInformationSummary {
    /// Enumerated part number for the device.
    pub part_number: Pn,
    /// Raw device revision field (lower 4 bits of register 0x25).
    pub device_revision: u8,
}

impl TryFrom<crate::field_sets::PartInformation> for PartInformationSummary {
    type Error = device_driver::ConversionError<u8>;

    fn try_from(value: crate::field_sets::PartInformation) -> Result<Self, Self::Error> {
        Ok(PartInformationSummary {
            part_number: value.pn()?,
            device_revision: value.dev_rev(),
        })
    }
}

/// High-level async driver for the BQ25887 charger.
pub struct Bq25887Driver<I2C: I2cTrait> {
    device: Bq25887<DeviceInterface<I2C>>,
}

impl<I2C: I2cTrait> Bq25887Driver<I2C> {
    /// Creates a new driver from the provided async I²C peripheral.
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_address(i2c, DEFAULT_I2C_ADDRESS)
    }

    /// Creates a new driver from the provided async I²C peripheral and custom 7-bit I²C address.
    pub fn new_with_address(i2c: I2C, address: u8) -> Self {
        Self {
            device: Bq25887::new(DeviceInterface::new(i2c, address)),
        }
    }

    /// Reads the cell voltage regulation limit register (0x00, reset = 0xA0).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_voltage_regulation_limit(
        &mut self,
    ) -> Result<crate::field_sets::CellVoltageLimit, BQ25887Error<I2C::Error>> {
        self.device.cell_voltage_limit().read_async().await
    }

    /// Writes `CellVoltageLimit` to the cell voltage regulation limit register (0x00, reset = 0xA0).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_voltage_regulation_limit(
        &mut self,
        volt_limit: crate::field_sets::CellVoltageLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .cell_voltage_limit()
            .write_async(|reg| *reg = volt_limit)
            .await
    }

    /// Reads the charger current limit register (0x01, reset = 0x5E).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_charge_current_limit(
        &mut self,
    ) -> Result<crate::field_sets::ChargeCurrentLimit, BQ25887Error<I2C::Error>> {
        self.device.charge_current_limit().read_async().await
    }

    /// Writes to the charger current limit register (0x01, reset = 0x5E).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_charge_current_limit(
        &mut self,
        current_limit: crate::field_sets::ChargeCurrentLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .charge_current_limit()
            .write_async(|reg| *reg = current_limit)
            .await
    }

    /// Enables or disables the battery connection by updating `EN_HIZ` in register 0x01.
    ///
    /// When `setting` is `true`, the battery connection is enabled (`EN_HIZ` cleared).
    /// When `setting` is `false`, the charger enters high-impedance mode (`EN_HIZ` set).
    ///
    /// # Errors
    ///
    /// Returns [`BQ25887Error::I2c`] if the underlying read or write of register 0x01 fails, or [`BQ25887Error::Conversion`] if the freshly read register image cannot be parsed.
    pub async fn enable_battery_connection(&mut self, setting: bool) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut reg = self.device.charge_current_limit().read_async().await?;
        reg.set_en_hiz(!setting);
        self.device.charge_current_limit().write_async(|w| *w = reg).await
    }

    /// Reads the input voltage limit register (0x02, reset = 0x84).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_input_voltage_limit(
        &mut self,
    ) -> Result<crate::field_sets::InputVoltageLimit, BQ25887Error<I2C::Error>> {
        self.device.input_voltage_limit().read_async().await
    }

    /// Writes `InputVoltageLimit` to the input voltage limit register (0x02, reset = 0x84).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_input_voltage_limit(
        &mut self,
        input_volt_limit: crate::field_sets::InputVoltageLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .input_voltage_limit()
            .write_async(|reg| *reg = input_volt_limit)
            .await
    }

    /// Reads the input current limit register (0x03, reset = 0x39).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_input_current_limit(
        &mut self,
    ) -> Result<crate::field_sets::InputCurrentLimit, BQ25887Error<I2C::Error>> {
        self.device.input_current_limit().read_async().await
    }

    /// Writes `InputCurrentLimit` to the input current limit register (0x03, reset = 0x39).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_input_current_limit(
        &mut self,
        current_limit: crate::field_sets::InputCurrentLimit,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .input_current_limit()
            .write_async(|reg| *reg = current_limit)
            .await
    }

    /// Reads the precharge and termination current limit register (0x04, reset = 0x22).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_precharge_and_termination_current_limit(
        &mut self,
    ) -> Result<crate::field_sets::PrechgTerminationCtrl, BQ25887Error<I2C::Error>> {
        self.device.prechg_termination_ctrl().read_async().await
    }

    /// Writes `PrechgTerminationCtrl` to the precharge and termination current limit register (0x04, reset = 0x22).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_precharge_and_termination_current_limit(
        &mut self,
        current_limit: crate::field_sets::PrechgTerminationCtrl,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .prechg_termination_ctrl()
            .write_async(|reg| *reg = current_limit)
            .await
    }

    /// Reads the charger control 1 register (0x05, reset = 0x9D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_control_1(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl1, BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_1().read_async().await
    }

    /// Writes `ChargerCtrl1` to the charger control 1 register (0x05, reset = 0x9D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_charger_control_1(
        &mut self,
        control: crate::field_sets::ChargerCtrl1,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_1().write_async(|reg| *reg = control).await
    }

    /// Reads the charger control 2 register (0x06, reset = 0x7D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_control_2(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl2, BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_2().read_async().await
    }

    /// Writes `ChargerCtrl2` to the charger control 2 register (0x06, reset = 0x7D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_charger_control_2(
        &mut self,
        control: crate::field_sets::ChargerCtrl2,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_2().write_async(|reg| *reg = control).await
    }

    /// Reads the charger control 3 register (0x07, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_control_3(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl3, BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_3().read_async().await
    }

    /// Writes `ChargerCtrl3` to the charger control 3 register (0x07, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_charger_control_3(
        &mut self,
        control: crate::field_sets::ChargerCtrl3,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_3().write_async(|reg| *reg = control).await
    }

    /// Reads the charger control 4 register (0x08, reset = 0x0D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_control_4(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl4, BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_4().read_async().await
    }

    /// Writes `ChargerCtrl4` to the charger control 4 register (0x08, reset = 0x0D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_charger_control_4(
        &mut self,
        control: crate::field_sets::ChargerCtrl4,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_4().write_async(|reg| *reg = control).await
    }

    /// Reads the ICO current limit register (0x0A).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ico_current_limit_in_use(
        &mut self,
    ) -> Result<crate::field_sets::IcoCurrentLimit, BQ25887Error<I2C::Error>> {
        self.device.ico_current_limit().read_async().await
    }

    /// Reads the charger status 1 register (0x0B).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_status_1(
        &mut self,
    ) -> Result<crate::field_sets::ChargerStatus1, BQ25887Error<I2C::Error>> {
        self.device.charger_status_1().read_async().await
    }

    /// Reads the charger status 2 register (0x0C).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_status_2(
        &mut self,
    ) -> Result<crate::field_sets::ChargerStatus2, BQ25887Error<I2C::Error>> {
        self.device.charger_status_2().read_async().await
    }

    /// Reads the NTC status register (0x0D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ntc_status(&mut self) -> Result<crate::field_sets::NtcStatus, BQ25887Error<I2C::Error>> {
        self.device.ntc_status().read_async().await
    }

    /// Reads the fault status register (0x0E).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_fault_status(&mut self) -> Result<crate::field_sets::FaultStatus, BQ25887Error<I2C::Error>> {
        self.device.fault_status().read_async().await
    }

    /// Reads the charger flag 1 register (0x0F).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_flag_1(&mut self) -> Result<crate::field_sets::ChargerFlag1, BQ25887Error<I2C::Error>> {
        self.device.charger_flag_1().read_async().await
    }

    /// Reads the charger flag 2 register (0x10).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_flag_2(&mut self) -> Result<crate::field_sets::ChargerFlag2, BQ25887Error<I2C::Error>> {
        self.device.charger_flag_2().read_async().await
    }

    /// Reads the fault flag register (0x11).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_fault_flag(&mut self) -> Result<crate::field_sets::FaultFlag, BQ25887Error<I2C::Error>> {
        self.device.fault_flag().read_async().await
    }

    /// Reads the charger mask 1 register (0x12, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_mask_1(&mut self) -> Result<crate::field_sets::ChargerMask1, BQ25887Error<I2C::Error>> {
        self.device.charger_mask_1().read_async().await
    }

    /// Writes `ChargerMask1` to the charger mask 1 register (0x12, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_charger_mask_1(
        &mut self,
        mask: crate::field_sets::ChargerMask1,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_mask_1().write_async(|reg| *reg = mask).await
    }

    /// Reads the charger mask 2 register (0x13, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_mask_2(&mut self) -> Result<crate::field_sets::ChargerMask2, BQ25887Error<I2C::Error>> {
        self.device.charger_mask_2().read_async().await
    }

    /// Writes `ChargerMask2` to the charger mask 2 register (0x13, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_charger_mask_2(
        &mut self,
        mask: crate::field_sets::ChargerMask2,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_mask_2().write_async(|reg| *reg = mask).await
    }

    /// Reads the fault mask register (0x14, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_fault_mask(&mut self) -> Result<crate::field_sets::FaultMask, BQ25887Error<I2C::Error>> {
        self.device.fault_mask().read_async().await
    }

    /// Writes `FaultMask` to the fault mask register (0x14, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_fault_mask(
        &mut self,
        mask: crate::field_sets::FaultMask,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.fault_mask().write_async(|reg| *reg = mask).await
    }

    /// Reads the ADC control register (0x15, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_adc_control(&mut self) -> Result<crate::field_sets::AdcControl, BQ25887Error<I2C::Error>> {
        self.device.adc_control().read_async().await
    }

    /// Writes `AdcControl` to the ADC control register (0x15, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_adc_control(
        &mut self,
        control: crate::field_sets::AdcControl,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.adc_control().write_async(|reg| *reg = control).await
    }

    /// Reads the ADC function disable register (0x16, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_adc_function_disable(
        &mut self,
    ) -> Result<crate::field_sets::AdcFunctionDisable, BQ25887Error<I2C::Error>> {
        self.device.adc_function_disable().read_async().await
    }

    /// Writes `AdcFunctionDisable` to the ADC function disable register (0x16, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn write_adc_function_disable(
        &mut self,
        function: crate::field_sets::AdcFunctionDisable,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device
            .adc_function_disable()
            .write_async(|reg| *reg = function)
            .await
    }

    /// Reads the IBUS ADC MSB register (0x17, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ibus_adc_1(&mut self) -> Result<crate::field_sets::IbusAdc1, BQ25887Error<I2C::Error>> {
        self.device.ibus_adc_1().read_async().await
    }

    /// Reads the IBUS ADC LSB register (0x18, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ibus_adc_0(&mut self) -> Result<crate::field_sets::IbusAdc0, BQ25887Error<I2C::Error>> {
        self.device.ibus_adc_0().read_async().await
    }

    /// Reads the ICHG ADC MSB register (0x19, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ichg_adc_1(&mut self) -> Result<crate::field_sets::IchgAdc1, BQ25887Error<I2C::Error>> {
        self.device.ichg_adc_1().read_async().await
    }

    /// Reads the ICHG ADC LSB register (0x1A, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ichg_adc_0(&mut self) -> Result<crate::field_sets::IchgAdc0, BQ25887Error<I2C::Error>> {
        self.device.ichg_adc_0().read_async().await
    }

    /// Reads the VBUS ADC MSB register (0x1B, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vbus_adc_1(&mut self) -> Result<crate::field_sets::VbusAdc1, BQ25887Error<I2C::Error>> {
        self.device.vbus_adc_1().read_async().await
    }

    /// Reads the VBUS ADC LSB register (0x1C, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vbus_adc_0(&mut self) -> Result<crate::field_sets::VbusAdc0, BQ25887Error<I2C::Error>> {
        self.device.vbus_adc_0().read_async().await
    }

    /// Reads the VBAT ADC MSB register (0x1D, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vbat_adc_1(&mut self) -> Result<crate::field_sets::VbatAdc1, BQ25887Error<I2C::Error>> {
        self.device.vbat_adc_1().read_async().await
    }

    /// Reads the VBAT ADC LSB register (0x1E, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vbat_adc_0(&mut self) -> Result<crate::field_sets::VbatAdc0, BQ25887Error<I2C::Error>> {
        self.device.vbat_adc_0().read_async().await
    }

    /// Reads the VCELLTOP ADC MSB register (0x1F, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vcell_top_adc_1(&mut self) -> Result<crate::field_sets::VcelltopAdc1, BQ25887Error<I2C::Error>> {
        self.device.vcelltop_adc_1().read_async().await
    }

    /// Reads the VCELLTOP ADC LSB register (0x20, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vcell_top_adc_0(&mut self) -> Result<crate::field_sets::VcelltopAdc0, BQ25887Error<I2C::Error>> {
        self.device.vcelltop_adc_0().read_async().await
    }

    /// Reads the TS ADC MSB register (0x21, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ts_adc_1(&mut self) -> Result<crate::field_sets::TsAdc1, BQ25887Error<I2C::Error>> {
        self.device.ts_adc_1().read_async().await
    }

    /// Reads the TS ADC LSB register (0x22, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ts_adc_0(&mut self) -> Result<crate::field_sets::TsAdc0, BQ25887Error<I2C::Error>> {
        self.device.ts_adc_0().read_async().await
    }

    /// Reads the TDIE ADC MSB register (0x23, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_tdie_adc_1(&mut self) -> Result<crate::field_sets::TdieAdc1, BQ25887Error<I2C::Error>> {
        self.device.tdie_adc_1().read_async().await
    }

    /// Reads the TDIE ADC LSB register (0x24, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_tdie_adc_0(&mut self) -> Result<crate::field_sets::TdieAdc0, BQ25887Error<I2C::Error>> {
        self.device.tdie_adc_0().read_async().await
    }

    /// Reads register 0x25 and returns parsed identification details.
    ///
    /// # Errors
    /// Returns an error if the I²C transaction fails.
    pub async fn read_part_information(&mut self) -> Result<PartInformationSummary, BQ25887Error<I2C::Error>> {
        let reg = self.device.part_information().read_async().await?;
        let summary = PartInformationSummary::try_from(reg).map_err(BQ25887Error::from)?;
        Ok(summary)
    }

    /// Issues a master reset by asserting `REG_RST` in register 0x25.
    ///
    /// # Errors
    ///
    /// Returns [`BQ25887Error::I2c`] if either the read or write of register 0x25 fails.
    pub async fn master_reset(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut reg = self.device.part_information().read_async().await?;
        reg.set_reg_rst(true);
        self.device.part_information().write_async(|r| *r = reg).await
    }

    /// ### Brief
    /// Reads VCELLBOT ADC 1 Register,
    /// (Address = 0x26) (reset = 0x00)
    /// BQ255887 p.64
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vcellbot_adc_1(&mut self) -> Result<crate::field_sets::VcellbotAdc1, BQ25887Error<I2C::Error>> {
        self.device.vcellbot_adc_1().read_async().await
    }

    /// ### Brief
    /// Reads VCELLBOT ADC 0 Register,
    /// (Address = 0x27) (reset = 0x00)
    /// BQ255887 p.64
    /// ### Errors
    /// Returns an error if the I²C transaction fails
    pub async fn read_vcellbot_adc_0(&mut self) -> Result<crate::field_sets::VcellbotAdc0, BQ25887Error<I2C::Error>> {
        self.device.vcellbot_adc_0().read_async().await
    }

    /// ### Brief
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

    /// ### Brief
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

    /// ### Brief
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

    /// ### Brief
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

    /// ### Brief
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

    /// ### Brief
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

    /// ### Brief
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

    /// ### Brief
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

    /// ### Brief
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

    /// ### Brief
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

    // ========================================================================
    // Interrupt and Flag Convenience Methods
    // ========================================================================

    /// Masks all interrupt sources, disabling INT pin pulses.
    ///
    /// Use this when the INT pin is not connected or not monitored. Events will
    /// still be recorded in the FLAG registers and can be read via
    /// [`read_and_clear_all_flags`].
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn mask_all_interrupts(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        // Mask all charger interrupts (CHARGER_MASK_1: 0x12)
        self.device.charger_mask_1().write_async(|reg| {
            reg.set_adc_done_mask(true);
            reg.set_iindpm_mask(true);
            reg.set_vindpm_mask(true);
            reg.set_treg_mask(true);
            reg.set_wd_mask(true);
            reg.set_chrg_mask(true);
        }).await?;

        // Mask all charger interrupts (CHARGER_MASK_2: 0x13)
        self.device.charger_mask_2().write_async(|reg| {
            reg.set_pg_mask(true);
            reg.set_vbus_mask(true);
            reg.set_ts_mask(true);
            reg.set_ico_mask(true);
        }).await?;

        // Mask all fault interrupts (FAULT_MASK: 0x14)
        self.device.fault_mask().write_async(|reg| {
            reg.set_vbus_ovp_mask(true);
            reg.set_tshut_mask(true);
            reg.set_tmr_mask(true);
            reg.set_sns_short_mask(true);
        }).await?;

        // Mask all cell balance interrupts (CELL_BALANCE_MASK: 0x2A)
        self.device.cell_balance_mask().write_async(|reg| {
            reg.set_cb_mask(true);
            reg.set_hs_cv_mask(true);
            reg.set_ls_cv_mask(true);
            reg.set_hs_ov_mask(true);
            reg.set_ls_ov_mask(true);
            reg.set_cb_oc_mask(true);
        }).await?;

        Ok(())
    }

    /// Reads and clears all FLAG registers, returning events that occurred since last read.
    ///
    /// FLAG registers are cleared on read, so this captures events that happened
    /// between polling cycles. Useful when INT pin is not connected.
    ///
    /// # Returns
    ///
    /// A tuple of (`charger_flag_1`, `charger_flag_2`, `fault_flag`, `cell_balance_flag`) registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_and_clear_all_flags(&mut self) -> Result<
        (
            crate::field_sets::ChargerFlag1,
            crate::field_sets::ChargerFlag2,
            crate::field_sets::FaultFlag,
            crate::field_sets::CellBalanceFlag,
        ),
        BQ25887Error<I2C::Error>
    > {
        let flag1 = self.device.charger_flag_1().read_async().await?;
        let flag2 = self.device.charger_flag_2().read_async().await?;
        let fault = self.device.fault_flag().read_async().await?;
        let cell_balance = self.device.cell_balance_flag().read_async().await?;
        Ok((flag1, flag2, fault, cell_balance))
    }

    // ========================================================================
    // Watchdog Timer Convenience Methods
    // ========================================================================

    /// Disables the I2C watchdog timer.
    ///
    /// When the watchdog is disabled, the device will remain in host mode indefinitely
    /// without requiring periodic watchdog resets. This is useful when the host cannot
    /// guarantee timely watchdog servicing.
    ///
    /// # Note
    ///
    /// When the watchdog timer expires, the device returns to default mode and resets
    /// most registers to their default values (including `ADC_CONTROL`, which disables
    /// the ADC). Disabling the watchdog prevents this automatic reset.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn disable_watchdog(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_1().modify_async(|reg| {
            reg.set_watchdog(crate::generated::Watchdog::WdDisable);
        }).await
    }

    /// Sets the I2C watchdog timer timeout period.
    ///
    /// The watchdog timer must be reset by calling [`reset_watchdog`] before it
    /// expires, otherwise the device will return to default mode and reset most
    /// registers to their default values.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The watchdog timeout period (40s, 80s, 160s, or disabled)
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn set_watchdog_timeout(
        &mut self,
        timeout: crate::generated::Watchdog,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_1().modify_async(|reg| {
            reg.set_watchdog(timeout);
        }).await
    }

    /// Resets the I2C watchdog timer.
    ///
    /// This must be called periodically (before the watchdog timeout expires) to
    /// keep the device in host mode. If the watchdog expires, the device returns
    /// to default mode and resets most registers to their default values.
    ///
    /// The `WD_RST` bit is self-clearing (returns to 0 after the reset is performed).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn reset_watchdog(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_3().modify_async(|reg| {
            reg.set_wd_rst(true);
        }).await
    }

    // ========================================================================
    // ADC Convenience Methods
    // ========================================================================

    /// Checks if the ADC is currently enabled.
    ///
    /// This reads the `ADC_EN` bit from the `ADC_CONTROL` register. It's recommended
    /// to verify `ADC_EN` after enabling the ADC, as the device may auto-clear it
    /// under certain conditions (e.g., no valid power source, all ADC channels
    /// disabled, or mode changes).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn is_adc_enabled(&mut self) -> Result<bool, BQ25887Error<I2C::Error>> {
        let adc_control = self.device.adc_control().read_async().await?;
        Ok(adc_control.adc_en())
    }

    /// Checks if power is good (input source detection complete).
    ///
    /// Returns true when `PG_STAT` is HIGH, indicating:
    /// - VBUS is above `VBUS_UVLO_RISING`
    /// - VBUS is below `VBUS_OV` threshold
    /// - Input is not a poor source
    /// - Input Source Type Detection is completed
    /// - CD pin is LOW
    ///
    /// ADC conversion is interrupted upon adapter plug-in and only resumes after
    /// Input Source Type Detection is complete. Check this before trusting ADC
    /// values after a plug event.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn is_power_good(&mut self) -> Result<bool, BQ25887Error<I2C::Error>> {
        let status = self.device.charger_status_2().read_async().await?;
        Ok(status.pg_stat())
    }

    /// Enables the ADC in continuous conversion mode.
    ///
    /// This configures the `ADC_CONTROL` register (0x15) to enable the ADC with
    /// continuous conversions. The ADC will continuously update voltage and
    /// current measurements.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn enable_adc_continuous(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.adc_control().write_async(|reg| {
            reg.set_adc_en(true);
            reg.set_adc_rate(crate::generated::AdcRate::Continuous);
        }).await
    }

    /// Enables the ADC in one-shot conversion mode.
    ///
    /// This configures the `ADC_CONTROL` register (0x15) to enable the ADC with
    /// one-shot conversion. The ADC will perform a single conversion.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn enable_adc_oneshot(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.adc_control().write_async(|reg| {
            reg.set_adc_en(true);
            reg.set_adc_rate(crate::generated::AdcRate::OneShot);
        }).await
    }

    /// Disables the ADC.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn disable_adc(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.adc_control().write_async(|reg| {
            reg.set_adc_en(false);
        }).await
    }

    /// Reads the battery voltage in millivolts.
    ///
    /// Combines the VBAT ADC MSB and LSB registers (0x1D, 0x1E) into a single
    /// voltage value. Resolution is 1mV per LSB.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vbat_mv(&mut self) -> Result<u16, BQ25887Error<I2C::Error>> {
        let msb = self.read_vbat_adc_1().await?;
        let lsb = self.read_vbat_adc_0().await?;
        let raw = (u16::from(msb.vbat_adc_msb()) << 8) | u16::from(lsb.vbat_adc_lsb());
        Ok(raw) // 1mV per LSB
    }

    /// Reads the VBUS (input) voltage in millivolts.
    ///
    /// Combines the VBUS ADC MSB and LSB registers (0x1B, 0x1C) into a single
    /// voltage value. Resolution is 1mV per LSB.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vbus_mv(&mut self) -> Result<u16, BQ25887Error<I2C::Error>> {
        let msb = self.read_vbus_adc_1().await?;
        let lsb = self.read_vbus_adc_0().await?;
        let raw = (u16::from(msb.vbus_adc_msb()) << 8) | u16::from(lsb.vbus_adc_lsb());
        Ok(raw) // 1mV per LSB
    }

    /// Reads the charge current in milliamps.
    ///
    /// Combines the ICHG ADC MSB and LSB registers (0x19, 0x1A) into a single
    /// current value. Resolution is 1mA per LSB.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ichg_ma(&mut self) -> Result<u16, BQ25887Error<I2C::Error>> {
        let msb = self.read_ichg_adc_1().await?;
        let lsb = self.read_ichg_adc_0().await?;
        let raw = (u16::from(msb.ichg_adc_msb()) << 8) | u16::from(lsb.ichg_adc_lsb());
        Ok(raw) // 1mA per LSB
    }

    /// Reads the input current in milliamps.
    ///
    /// Combines the IBUS ADC MSB and LSB registers (0x17, 0x18) into a single
    /// current value. Resolution is 1mA per LSB.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ibus_ma(&mut self) -> Result<u16, BQ25887Error<I2C::Error>> {
        let msb = self.read_ibus_adc_1().await?;
        let lsb = self.read_ibus_adc_0().await?;
        let raw = (u16::from(msb.ibus_adc_msb()) << 8) | u16::from(lsb.ibus_adc_lsb());
        Ok(raw) // 1mA per LSB
    }

    /// Reads the top cell voltage in millivolts (for 2S configurations).
    ///
    /// Combines the VCELLTOP ADC MSB and LSB registers (0x1F, 0x20) into a single
    /// voltage value. Resolution is 1mV per LSB.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vcell_top_mv(&mut self) -> Result<u16, BQ25887Error<I2C::Error>> {
        let msb = self.read_vcell_top_adc_1().await?;
        let lsb = self.read_vcell_top_adc_0().await?;
        let raw = (u16::from(msb.vcelltop_adc_msb()) << 8) | u16::from(lsb.vcelltop_adc_lsb());
        Ok(raw) // 1mV per LSB
    }

    /// Reads the bottom cell voltage in millivolts (for 2S configurations).
    ///
    /// Combines the VCELLBOT ADC MSB and LSB registers (0x26, 0x27) into a single
    /// voltage value. Resolution is 1mV per LSB.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_vcell_bot_mv(&mut self) -> Result<u16, BQ25887Error<I2C::Error>> {
        let msb = self.read_vcellbot_adc_1().await?;
        let lsb = self.read_vcellbot_adc_0().await?;
        let raw = (u16::from(msb.vcellbot_adc_msb()) << 8) | u16::from(lsb.vcellbot_adc_lsb());
        Ok(raw) // 1mV per LSB
    }

    /// Reads the die temperature ADC raw value.
    ///
    /// Combines the TDIE ADC MSB and LSB registers (0x23, 0x24) into a single
    /// raw ADC value. Use [`read_tdie_celsius`] for a converted temperature value.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_tdie_raw(&mut self) -> Result<u16, BQ25887Error<I2C::Error>> {
        let msb = self.read_tdie_adc_1().await?;
        let lsb = self.read_tdie_adc_0().await?;
        let raw = (u16::from(msb.tdie_adc_msb()) << 8) | u16::from(lsb.tdie_adc_lsb());
        Ok(raw)
    }

    /// Reads the die temperature in degrees Celsius (as fixed-point × 10).
    ///
    /// Returns the temperature multiplied by 10 to preserve 0.5°C resolution
    /// without floating point. For example, 25.5°C is returned as 255.
    ///
    /// The TDIE ADC has 0.5°C resolution with range 0°C to 128°C.
    /// Bit 15 is the sign bit (two's complement).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_tdie_decidegrees(&mut self) -> Result<i16, BQ25887Error<I2C::Error>> {
        let raw = self.read_tdie_raw().await?;
        // Each LSB = 0.5°C, so multiply by 5 to get decidegrees (0.1°C units)
        // Handle sign bit (bit 15) for two's complement
        let temp = if raw & 0x8000 != 0 {
            // Negative temperature (two's complement)
            -((((!raw).wrapping_add(1) & 0x7FFF).cast_signed()) * 5)
        } else {
            raw.cast_signed() * 5
        };
        Ok(temp)
    }

    /// Reads the thermistor (TS) ADC raw value.
    ///
    /// Combines the TS ADC MSB and LSB registers (0x21, 0x22) into a single
    /// raw ADC value. See datasheet for temperature conversion.
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ts_raw(&mut self) -> Result<u16, BQ25887Error<I2C::Error>> {
        let msb = self.read_ts_adc_1().await?;
        let lsb = self.read_ts_adc_0().await?;
        let raw = (u16::from(msb.ts_adc_msb()) << 8) | u16::from(lsb.ts_adc_lsb());
        Ok(raw)
    }

    // ========================================================================
    // Charging Control Convenience Methods
    // ========================================================================

    /// Sets the cell recharge threshold offset below VCELLREG.
    ///
    /// After charge termination, the charger automatically restarts when the
    /// battery voltage drops below (VCELLREG - threshold). A larger threshold
    /// gives more margin for automatic recharge to trigger.
    ///
    /// # Arguments
    ///
    /// * `threshold` - The recharge threshold (50mV, 100mV, 150mV, or 200mV)
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn set_recharge_threshold(
        &mut self,
        threshold: crate::generated::VcellRechg,
    ) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_2().modify_async(|reg| {
            reg.set_vcell_rechg(threshold);
        }).await
    }

    /// Checks if charging is enabled (`EN_CHG` bit).
    ///
    /// Returns true if the `EN_CHG` bit in `CHARGER_CTRL_2` is set, meaning
    /// the charger is allowed to charge (subject to other conditions like
    /// valid input source, no faults, etc.).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn is_charging_enabled(&mut self) -> Result<bool, BQ25887Error<I2C::Error>> {
        let ctrl2 = self.device.charger_ctrl_2().read_async().await?;
        Ok(ctrl2.en_chg())
    }

    /// Enables or disables charging.
    ///
    /// Sets the `EN_CHG` bit in `CHARGER_CTRL_2`. When disabled, the charger
    /// will not charge even if a valid input source is present.
    ///
    /// # Arguments
    ///
    /// * `enabled` - true to enable charging, false to disable
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn set_charging_enabled(&mut self, enabled: bool) -> Result<(), BQ25887Error<I2C::Error>> {
        self.device.charger_ctrl_2().modify_async(|reg| {
            reg.set_en_chg(enabled);
        }).await
    }

    /// Gets the current charge status.
    ///
    /// Returns the `CHRG_STAT` field from `CHARGER_STATUS_1`, indicating the
    /// current charging phase:
    /// - `NotCharging`: Not charging
    /// - `TrickleCharge`: Trickle charge (VBAT < `VBAT_SHORT`)
    /// - `PreCharge`: Pre-charge (`VBAT_UVLO` < VBAT < `VBAT_LOWV`)
    /// - `FastCharge`: Fast-charge (CC mode)
    /// - `TaperCharge`: Taper charge (CV mode)
    /// - `TopoffTimerCharging`: Top-off timer charging
    /// - `ChargeTerminationDone`: Charge termination done
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn get_charge_status(&mut self) -> Result<crate::generated::ChrgStat, BQ25887Error<I2C::Error>> {
        let status1 = self.device.charger_status_1().read_async().await?;
        Ok(status1.chrg_stat())
    }
}
