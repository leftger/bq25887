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
//! use bq25887::Bq25887Driver;
//!
//! async fn inspect<I2C>(i2c: I2C) -> Result<(), bq25887::BQ25887Error<I2C::Error>>
//! where
//!     I2C: embedded_hal_async::i2c::I2c,
//! {
//!     let mut driver = Bq25887Driver::new(i2c);
//!     let status = driver.read_charger_status_1().await?;
//!     // Evaluate charger status or update application state here.
//!     let _ = status;
//!     Ok(())
//! }
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
/// Generated register field definitions for the charger.
pub use generated::field_sets;

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

/// Cached configuration register values for addresses 0x00 through 0x06.
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, Default)]
pub struct ConfigurationCache {
    /// Cached value of register 0x00 (`CellVoltageLimit`) if observed.
    pub cell_voltage_limit: Option<crate::field_sets::CellVoltageLimit>,
    /// Cached value of register 0x01 (`ChargeCurrentLimit`) if observed.
    pub charge_current_limit: Option<crate::field_sets::ChargeCurrentLimit>,
    /// Cached value of register 0x02 (`InputVoltageLimit`) if observed.
    pub input_voltage_limit: Option<crate::field_sets::InputVoltageLimit>,
    /// Cached value of register 0x03 (`InputCurrentLimit`) if observed.
    pub input_current_limit: Option<crate::field_sets::InputCurrentLimit>,
    /// Cached value of register 0x04 (`PrechgTerminationCtrl`) if observed.
    pub precharge_termination_control: Option<crate::field_sets::PrechgTerminationCtrl>,
    /// Cached value of register 0x05 (`ChargerCtrl1`) if observed.
    pub charger_control_1: Option<crate::field_sets::ChargerCtrl1>,
    /// Cached value of register 0x06 (`ChargerCtrl2`) if observed.
    pub charger_control_2: Option<crate::field_sets::ChargerCtrl2>,
}

/// Cached status and fault register values observed by the driver.
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, Default)]
pub struct StatusCache {
    /// Cached value of register 0x0B (`ChargerStatus1`) if observed.
    pub charger_status_1: Option<crate::field_sets::ChargerStatus1>,
    /// Cached value of register 0x0C (`ChargerStatus2`) if observed.
    pub charger_status_2: Option<crate::field_sets::ChargerStatus2>,
    /// Cached value of register 0x0D (`NtcStatus`) if observed.
    pub ntc_status: Option<crate::field_sets::NtcStatus>,
    /// Cached value of register 0x0E (`FaultStatus`) if observed.
    pub fault_status: Option<crate::field_sets::FaultStatus>,
    /// Cached value of register 0x0F (`ChargerFlag1`) if observed.
    pub charger_flag_1: Option<crate::field_sets::ChargerFlag1>,
    /// Cached value of register 0x10 (`ChargerFlag2`) if observed.
    pub charger_flag_2: Option<crate::field_sets::ChargerFlag2>,
    /// Cached value of register 0x11 (`FaultFlag`) if observed.
    pub fault_flag: Option<crate::field_sets::FaultFlag>,
    /// Cached value of register 0x12 (`ChargerMask1`) if observed.
    pub charger_mask_1: Option<crate::field_sets::ChargerMask1>,
    /// Cached value of register 0x13 (`ChargerMask2`) if observed.
    pub charger_mask_2: Option<crate::field_sets::ChargerMask2>,
    /// Cached value of register 0x14 (`FaultMask`) if observed.
    pub fault_mask: Option<crate::field_sets::FaultMask>,
}

/// High-level async driver for the BQ25887 charger.
pub struct Bq25887Driver<I2C: I2cTrait> {
    device: Bq25887<DeviceInterface<I2C>>,
    config_cache: ConfigurationCache,
    status_cache: StatusCache,
    part_information_cache: Option<PartInformationSummary>,
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
            config_cache: ConfigurationCache::default(),
            status_cache: StatusCache::default(),
            part_information_cache: None,
        }
    }

    /// Returns the cached configuration register values observed by the driver.
    pub fn configuration_cache(&self) -> &ConfigurationCache {
        &self.config_cache
    }

    /// Returns the cached status and fault register values observed by the driver.
    pub fn status_cache(&self) -> &StatusCache {
        &self.status_cache
    }

    /// Refreshes the cached configuration registers (0x00 through 0x06) by issuing reads to the device.
    ///
    /// # Errors
    /// Returns an error if any I²C transaction fails or if a register value cannot be parsed.
    pub async fn refresh_configuration_cache(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        let voltage_limit = self.device.cell_voltage_limit().read_async().await?;
        self.config_cache.cell_voltage_limit = Some(voltage_limit);
        let charge_current_limit = self.device.charge_current_limit().read_async().await?;
        self.config_cache.charge_current_limit = Some(charge_current_limit);
        let input_voltage_limit = self.device.input_voltage_limit().read_async().await?;
        self.config_cache.input_voltage_limit = Some(input_voltage_limit);
        let input_current_limit = self.device.input_current_limit().read_async().await?;
        self.config_cache.input_current_limit = Some(input_current_limit);
        let prechg_term_ctrl = self.device.prechg_termination_ctrl().read_async().await?;
        self.config_cache.precharge_termination_control = Some(prechg_term_ctrl);
        let charger_control_1 = self.device.charger_ctrl_1().read_async().await?;
        self.config_cache.charger_control_1 = Some(charger_control_1);
        let charger_control_2 = self.device.charger_ctrl_2().read_async().await?;
        self.config_cache.charger_control_2 = Some(charger_control_2);
        Ok(())
    }

    /// Refreshes the cached status and fault registers (0x0B through 0x14) by issuing reads to the device.
    ///
    /// # Errors
    /// Returns an error if any I²C transaction fails or if a register value cannot be parsed.
    pub async fn refresh_status_register_cache(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        let charger_status_1 = self.device.charger_status_1().read_async().await?;
        self.status_cache.charger_status_1 = Some(charger_status_1);
        let charger_status_2 = self.device.charger_status_2().read_async().await?;
        self.status_cache.charger_status_2 = Some(charger_status_2);
        let ntc_status = self.device.ntc_status().read_async().await?;
        self.status_cache.ntc_status = Some(ntc_status);
        let fault_status = self.device.fault_status().read_async().await?;
        self.status_cache.fault_status = Some(fault_status);
        let charger_flag_1 = self.device.charger_flag_1().read_async().await?;
        self.status_cache.charger_flag_1 = Some(charger_flag_1);
        let charger_flag_2 = self.device.charger_flag_2().read_async().await?;
        self.status_cache.charger_flag_2 = Some(charger_flag_2);
        let fault_flag = self.device.fault_flag().read_async().await?;
        self.status_cache.fault_flag = Some(fault_flag);
        let charger_mask_1 = self.device.charger_mask_1().read_async().await?;
        self.status_cache.charger_mask_1 = Some(charger_mask_1);
        let charger_mask_2 = self.device.charger_mask_2().read_async().await?;
        self.status_cache.charger_mask_2 = Some(charger_mask_2);
        let fault_mask = self.device.fault_mask().read_async().await?;
        self.status_cache.fault_mask = Some(fault_mask);
        Ok(())
    }

    /// Reads the cell voltage regulation limit register (0x00, reset = 0xA0).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_voltage_regulation_limit(
        &mut self,
    ) -> Result<crate::field_sets::CellVoltageLimit, BQ25887Error<I2C::Error>> {
        let value = self.device.cell_voltage_limit().read_async().await?;
        self.config_cache.cell_voltage_limit = Some(value);
        Ok(value)
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
            .await?;
        self.config_cache.cell_voltage_limit = Some(volt_limit);
        Ok(())
    }

    /// Reads the charger current limit register (0x01, reset = 0x5E).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_charge_current_limit(
        &mut self,
    ) -> Result<crate::field_sets::ChargeCurrentLimit, BQ25887Error<I2C::Error>> {
        let value = self.device.charge_current_limit().read_async().await?;
        self.config_cache.charge_current_limit = Some(value);
        Ok(value)
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
            .await?;
        self.config_cache.charge_current_limit = Some(current_limit);
        Ok(())
    }

    /// Enables or disables the battery connection by updating `EN_HIZ` in register 0x01.
    ///
    /// When `setting` is `true`, the battery connection is enabled (`EN_HIZ` cleared).
    /// When `setting` is `false`, the charger enters high-impedance mode (`EN_HIZ` set).
    /// The method reuses the cached register value when available to minimize I²C traffic.
    ///
    /// # Errors
    ///
    /// Returns [`BQ25887Error::I2c`] if the underlying read or write of register 0x01 fails, or [`BQ25887Error::Conversion`] if the freshly read register image cannot be parsed.
    pub async fn enable_battery_connection(&mut self, setting: bool) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut reg = if let Some(cached) = self.config_cache.charge_current_limit {
            cached
        } else {
            self.device.charge_current_limit().read_async().await?
        };
        reg.set_en_hiz(!setting);
        self.device.charge_current_limit().write_async(|w| *w = reg).await?;
        self.config_cache.charge_current_limit = Some(reg);
        Ok(())
    }

    /// Reads the input voltage limit register (0x02, reset = 0x84).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_input_voltage_limit(
        &mut self,
    ) -> Result<crate::field_sets::InputVoltageLimit, BQ25887Error<I2C::Error>> {
        let value = self.device.input_voltage_limit().read_async().await?;
        self.config_cache.input_voltage_limit = Some(value);
        Ok(value)
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
            .await?;
        self.config_cache.input_voltage_limit = Some(input_volt_limit);
        Ok(())
    }

    /// Reads the input current limit register (0x03, reset = 0x39).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_input_current_limit(
        &mut self,
    ) -> Result<crate::field_sets::InputCurrentLimit, BQ25887Error<I2C::Error>> {
        let value = self.device.input_current_limit().read_async().await?;
        self.config_cache.input_current_limit = Some(value);
        Ok(value)
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
            .await?;
        self.config_cache.input_current_limit = Some(current_limit);
        Ok(())
    }

    /// Reads the precharge and termination current limit register (0x04, reset = 0x22).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails or the register value cannot be parsed.
    pub async fn read_precharge_and_termination_current_limit(
        &mut self,
    ) -> Result<crate::field_sets::PrechgTerminationCtrl, BQ25887Error<I2C::Error>> {
        let value = self.device.prechg_termination_ctrl().read_async().await?;
        self.config_cache.precharge_termination_control = Some(value);
        Ok(value)
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
            .await?;
        self.config_cache.precharge_termination_control = Some(current_limit);
        Ok(())
    }

    /// Reads the charger control 1 register (0x05, reset = 0x9D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_control_1(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl1, BQ25887Error<I2C::Error>> {
        let value = self.device.charger_ctrl_1().read_async().await?;
        self.config_cache.charger_control_1 = Some(value);
        Ok(value)
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
        self.device.charger_ctrl_1().write_async(|reg| *reg = control).await?;
        self.config_cache.charger_control_1 = Some(control);
        Ok(())
    }

    /// Reads the charger control 2 register (0x06, reset = 0x7D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_control_2(
        &mut self,
    ) -> Result<crate::field_sets::ChargerCtrl2, BQ25887Error<I2C::Error>> {
        let value = self.device.charger_ctrl_2().read_async().await?;
        self.config_cache.charger_control_2 = Some(value);
        Ok(value)
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
        self.device.charger_ctrl_2().write_async(|reg| *reg = control).await?;
        self.config_cache.charger_control_2 = Some(control);
        Ok(())
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
        if let Some(cached) = self.status_cache.charger_status_1 {
            return Ok(cached);
        }
        let value = self.device.charger_status_1().read_async().await?;
        self.status_cache.charger_status_1 = Some(value);
        Ok(value)
    }

    /// Reads the charger status 2 register (0x0C).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_status_2(
        &mut self,
    ) -> Result<crate::field_sets::ChargerStatus2, BQ25887Error<I2C::Error>> {
        if let Some(cached) = self.status_cache.charger_status_2 {
            return Ok(cached);
        }
        let value = self.device.charger_status_2().read_async().await?;
        self.status_cache.charger_status_2 = Some(value);
        Ok(value)
    }

    /// Reads the NTC status register (0x0D).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_ntc_status(&mut self) -> Result<crate::field_sets::NtcStatus, BQ25887Error<I2C::Error>> {
        if let Some(cached) = self.status_cache.ntc_status {
            return Ok(cached);
        }
        let value = self.device.ntc_status().read_async().await?;
        self.status_cache.ntc_status = Some(value);
        Ok(value)
    }

    /// Reads the fault status register (0x0E).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_fault_status(&mut self) -> Result<crate::field_sets::FaultStatus, BQ25887Error<I2C::Error>> {
        if let Some(cached) = self.status_cache.fault_status {
            return Ok(cached);
        }
        let value = self.device.fault_status().read_async().await?;
        self.status_cache.fault_status = Some(value);
        Ok(value)
    }

    /// Reads the charger flag 1 register (0x0F).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_flag_1(&mut self) -> Result<crate::field_sets::ChargerFlag1, BQ25887Error<I2C::Error>> {
        if let Some(cached) = self.status_cache.charger_flag_1 {
            return Ok(cached);
        }
        let value = self.device.charger_flag_1().read_async().await?;
        self.status_cache.charger_flag_1 = Some(value);
        Ok(value)
    }

    /// Reads the charger flag 2 register (0x10).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_flag_2(&mut self) -> Result<crate::field_sets::ChargerFlag2, BQ25887Error<I2C::Error>> {
        if let Some(cached) = self.status_cache.charger_flag_2 {
            return Ok(cached);
        }
        let value = self.device.charger_flag_2().read_async().await?;
        self.status_cache.charger_flag_2 = Some(value);
        Ok(value)
    }

    /// Reads the fault flag register (0x11).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_fault_flag(&mut self) -> Result<crate::field_sets::FaultFlag, BQ25887Error<I2C::Error>> {
        if let Some(cached) = self.status_cache.fault_flag {
            return Ok(cached);
        }
        let value = self.device.fault_flag().read_async().await?;
        self.status_cache.fault_flag = Some(value);
        Ok(value)
    }

    /// Reads the charger mask 1 register (0x12, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_mask_1(&mut self) -> Result<crate::field_sets::ChargerMask1, BQ25887Error<I2C::Error>> {
        if let Some(cached) = self.status_cache.charger_mask_1 {
            return Ok(cached);
        }
        let value = self.device.charger_mask_1().read_async().await?;
        self.status_cache.charger_mask_1 = Some(value);
        Ok(value)
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
        self.device.charger_mask_1().write_async(|reg| *reg = mask).await?;
        self.status_cache.charger_mask_1 = Some(mask);
        Ok(())
    }

    /// Reads the charger mask 2 register (0x13, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_charger_mask_2(&mut self) -> Result<crate::field_sets::ChargerMask2, BQ25887Error<I2C::Error>> {
        if let Some(cached) = self.status_cache.charger_mask_2 {
            return Ok(cached);
        }
        let value = self.device.charger_mask_2().read_async().await?;
        self.status_cache.charger_mask_2 = Some(value);
        Ok(value)
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
        self.device.charger_mask_2().write_async(|reg| *reg = mask).await?;
        self.status_cache.charger_mask_2 = Some(mask);
        Ok(())
    }

    /// Reads the fault mask register (0x14, reset = 0x00).
    ///
    /// # Errors
    ///
    /// Returns an error if the I²C transaction fails.
    pub async fn read_fault_mask(&mut self) -> Result<crate::field_sets::FaultMask, BQ25887Error<I2C::Error>> {
        if let Some(cached) = self.status_cache.fault_mask {
            return Ok(cached);
        }
        let value = self.device.fault_mask().read_async().await?;
        self.status_cache.fault_mask = Some(value);
        Ok(value)
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
        self.device.fault_mask().write_async(|reg| *reg = mask).await?;
        self.status_cache.fault_mask = Some(mask);
        Ok(())
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
        self.part_information_cache = Some(summary);
        Ok(summary)
    }

    /// Issues a master reset by asserting `REG_RST` in register 0x25.
    /// The method preserves the reported part number, writes the reset command to the device, and clears the driver's configuration cache so subsequent reads are refreshed.
    ///
    /// # Errors
    ///
    /// Returns [`BQ25887Error::I2c`] if either the read or write of register 0x25 fails.
    pub async fn master_reset(&mut self) -> Result<(), BQ25887Error<I2C::Error>> {
        let mut reg = self.device.part_information().read_async().await?;
        reg.set_reg_rst(true);
        self.device.part_information().write_async(|r| *r = reg).await?;
        self.config_cache = ConfigurationCache::default();
        self.status_cache = StatusCache::default();
        self.part_information_cache = None;
        Ok(())
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
}
