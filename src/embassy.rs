#![cfg(feature = "embassy")]
#![doc = "Embassy integration helpers for the BQ25887 driver."]

use embassy_embedded_hal::shared_bus::asynch::i2c::{I2cDevice, I2cDeviceWithConfig};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal_async::i2c::I2c;

use crate::Bq25887Driver;

/// Alias for the shared Embassy mutex that guards an async I²C peripheral.
pub type SharedBus<M, BUS> = Mutex<M, BUS>;

/// Convenience alias for a BQ25887 driver that operates on an Embassy shared-bus device.
pub type SharedDriver<'a, M, BUS> = Bq25887Driver<I2cDevice<'a, M, BUS>>;

/// Convenience alias for a BQ25887 driver that applies a per-device bus configuration
/// before every transaction.
pub type ConfiguredDriver<'a, M, BUS> = Bq25887Driver<I2cDeviceWithConfig<'a, M, BUS>>;

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
    Bq25887Driver::new(I2cDevice::new(bus))
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
    Bq25887Driver::new(I2cDeviceWithConfig::new(bus, config))
}
