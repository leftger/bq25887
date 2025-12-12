#![cfg(feature = "embassy")]
#![doc = "Embassy integration helpers for the BQ25887 driver."]

use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_embedded_hal::shared_bus::asynch::i2c::{I2cDevice, I2cDeviceWithConfig};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal_async::i2c::{ErrorType, I2c};

use crate::{BQ25887Error, Bq25887Driver, DEFAULT_I2C_ADDRESS};

/// Alias for the shared Embassy mutex that guards an async I²C peripheral.
pub type SharedBus<M, BUS> = Mutex<M, BUS>;

/// Convenience alias for a BQ25887 driver that operates on an Embassy shared-bus device.
pub type SharedDriver<'a, M, BUS> = Bq25887Driver<I2cDevice<'a, M, BUS>>;

/// Convenience alias for a BQ25887 driver that applies a per-device bus configuration
/// before every transaction.
pub type ConfiguredDriver<'a, M, BUS> = Bq25887Driver<I2cDeviceWithConfig<'a, M, BUS>>;

/// Error type produced when operating a [`SharedDriver`].
pub type SharedDriverError<BUS> = BQ25887Error<I2cDeviceError<<BUS as ErrorType>::Error>>;

/// Result alias returned by helpers that construct a [`SharedDriver`].
pub type SharedDriverResult<'a, M, BUS> = Result<SharedDriver<'a, M, BUS>, SharedDriverError<BUS>>;

/// Error type produced when operating a [`ConfiguredDriver`].
pub type ConfiguredDriverError<BUS> = SharedDriverError<BUS>;

/// Result alias returned by helpers that construct a [`ConfiguredDriver`].
pub type ConfiguredDriverResult<'a, M, BUS> = Result<ConfiguredDriver<'a, M, BUS>, ConfiguredDriverError<BUS>>;

/// Construct a `Bq25887Driver` from an Embassy shared I²C bus.
///
/// This helper wraps [`Bq25887Driver::new`] while hiding the verbose shared-bus types.
///
/// # Example
///
/// ```ignore
/// use bq25887::embassy::{new_driver, SharedBus};
/// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
/// use embassy_sync::mutex::Mutex;
///
/// static BUS: SharedBus<ThreadModeRawMutex, MyAsyncI2c> = Mutex::new(MyAsyncI2c::new());
///
/// async fn init() {
///     let mut driver = new_driver(&BUS);
///     driver.read_charger_status_1().await.unwrap();
/// }
/// ```
pub fn new_driver<'a, M, BUS>(bus: &'a SharedBus<M, BUS>) -> SharedDriver<'a, M, BUS>
where
    M: RawMutex + 'a,
    BUS: I2c + 'a,
{
    new_driver_with_address(bus, DEFAULT_I2C_ADDRESS)
}

/// Construct a `Bq25887Driver` from an Embassy shared I²C bus using a custom 7-bit I²C address.
pub fn new_driver_with_address<'a, M, BUS>(bus: &'a SharedBus<M, BUS>, address: u8) -> SharedDriver<'a, M, BUS>
where
    M: RawMutex + 'a,
    BUS: I2c + 'a,
{
    Bq25887Driver::new_with_address(I2cDevice::new(bus), address)
}

/// Construct a `Bq25887Driver` from an Embassy shared I²C bus and immediately populate the status cache.
///
/// This helper performs [`Bq25887Driver::refresh_status_register_cache`] after creating the driver,
/// ensuring that [`Bq25887Driver::status_cache`] returns populated data.
///
/// # Errors
///
/// Returns an error if any of the status register reads fail.
pub async fn new_driver_with_status_cache<'a, M, BUS>(bus: &'a SharedBus<M, BUS>) -> SharedDriverResult<'a, M, BUS>
where
    M: RawMutex + 'a,
    BUS: I2c + 'a,
{
    new_driver_with_status_cache_and_address(bus, DEFAULT_I2C_ADDRESS).await
}

/// Construct a shared-bus `Bq25887Driver` with a custom address and eagerly refreshed status cache.
pub async fn new_driver_with_status_cache_and_address<'a, M, BUS>(
    bus: &'a SharedBus<M, BUS>,
    address: u8,
) -> SharedDriverResult<'a, M, BUS>
where
    M: RawMutex + 'a,
    BUS: I2c + 'a,
{
    let mut driver = new_driver_with_address(bus, address);
    driver.refresh_status_register_cache().await?;
    Ok(driver)
}

/// Construct a `Bq25887Driver` that enforces a per-device bus configuration.
///
/// This is useful when multiple peripherals on the same bus require distinct timing
/// parameters. The provided `config` value is applied before every I²C operation.
///
/// # Example
///
/// ```ignore
/// use bq25887::embassy::new_driver_with_config;
/// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
///
/// let mut driver = new_driver_with_config(&BUS, MyAsyncI2cConfig::fast_mode());
/// ```
///
/// The `BUS` mutex must guard an async I²C peripheral whose type implements
/// both `embedded_hal_async::i2c::I2c` and `embassy_embedded_hal::SetConfig`.
pub fn new_driver_with_config<'a, M, BUS>(
    bus: &'a SharedBus<M, BUS>,
    config: BUS::Config,
) -> ConfiguredDriver<'a, M, BUS>
where
    M: RawMutex + 'a,
    BUS: I2c + embassy_embedded_hal::SetConfig + 'a,
{
    new_driver_with_config_and_address(bus, config, DEFAULT_I2C_ADDRESS)
}

/// Construct a configured `Bq25887Driver` for a shared I²C bus using a custom 7-bit address.
pub fn new_driver_with_config_and_address<'a, M, BUS>(
    bus: &'a SharedBus<M, BUS>,
    config: BUS::Config,
    address: u8,
) -> ConfiguredDriver<'a, M, BUS>
where
    M: RawMutex + 'a,
    BUS: I2c + embassy_embedded_hal::SetConfig + 'a,
{
    Bq25887Driver::new_with_address(I2cDeviceWithConfig::new(bus, config), address)
}

/// Construct a `Bq25887Driver` with per-device bus configuration and an eagerly refreshed status cache.
///
/// This helper mirrors [`new_driver_with_status_cache`] for devices that require dynamic bus configuration.
///
/// # Errors
///
/// Returns an error if refreshing the status registers fails.
pub async fn new_driver_with_config_and_status_cache<'a, M, BUS>(
    bus: &'a SharedBus<M, BUS>,
    config: BUS::Config,
) -> ConfiguredDriverResult<'a, M, BUS>
where
    M: RawMutex + 'a,
    BUS: I2c + embassy_embedded_hal::SetConfig + 'a,
{
    new_driver_with_config_and_status_cache_and_address(bus, config, DEFAULT_I2C_ADDRESS).await
}

/// Construct a configured `Bq25887Driver` with a custom address and eagerly refreshed status cache.
pub async fn new_driver_with_config_and_status_cache_and_address<'a, M, BUS>(
    bus: &'a SharedBus<M, BUS>,
    config: BUS::Config,
    address: u8,
) -> ConfiguredDriverResult<'a, M, BUS>
where
    M: RawMutex + 'a,
    BUS: I2c + embassy_embedded_hal::SetConfig + 'a,
{
    let mut driver = new_driver_with_config_and_address(bus, config, address);
    driver.refresh_status_register_cache().await?;
    Ok(driver)
}
