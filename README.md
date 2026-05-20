# `bq25887`

Platform-agnostic Rust driver for the Texas Instruments BQ25887 synchronous battery charger. The crate targets `no_std` environments, provides an async-friendly I²C interface built on `embedded-hal-async`, and generates its register API at build time from a YAML manifest.

---

## Features

- Asynchronous I²C driver (`Bq25887Driver`) built on `embedded-hal-async::i2c::I2c`.
- Complete register coverage through generated accessors (see `src/bq25887.yaml`).
- Convenience methods for common charger configuration flows.
- Optional logging integrations via the `defmt-03` and `log` feature flags.
- `no_std` compatible (only depends on `alloc`-free crates by default).

---

## Getting Started

Add the crate to your `Cargo.toml`:

```/dev/null/Cargo.toml#L1-9
[dependencies]
bq25887 = "0.1"
embedded-hal-async = "1.0"

# Optional feature flags:
# bq25887 = { version = "0.1", features = ["defmt-03"] }
# bq25887 = { version = "0.1", features = ["log"] }
```

---

## Async usage

Use any I²C type that implements `embedded_hal_async::i2c::I2c` to talk to the charger:

```rust
use bq25887::Bq25887Driver;

pub async fn configure_charger<I2C>(i2c: I2C) -> Result<(), bq25887::BQ25887Error<I2C::Error>>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    let mut driver = Bq25887Driver::new(i2c);

    // Read the configured cell charge voltage limit as a millivolt value
    let limit_mv = driver.read_cell_voltage_limit_mv().await?;

    // Check charger status
    let status = driver.read_charger_status_1().await?;
    let _ = (limit_mv, status);

    Ok(())
}
```

---

## Charger configuration helpers

The driver exposes high-level methods for each register group, e.g. `read_charge_current_limit`, `write_input_voltage_limit`, and friends. These helpers wrap the generated register accessors and convert conversion errors into the crate’s custom error type.

### ADC reading helpers

Convenience methods that combine MSB/LSB register pairs and return physical units:

| Method | Returns | Resolution |
|---|---|---|
| `read_vbat_mv` | battery voltage (mV) | 1 mV/LSB |
| `read_vbus_mv` | input voltage (mV) | 1 mV/LSB |
| `read_ichg_ma` | charge current (mA) | 1 mA/LSB |
| `read_ibus_ma` | input current (mA) | 1 mA/LSB |
| `read_vcell_top_mv` | top cell voltage (mV) | 1 mV/LSB |
| `read_vcell_bot_mv` | bottom cell voltage (mV) | 1 mV/LSB |
| `read_tdie_decidegrees` | die temperature (0.1 °C units) | 0.5 °C/LSB |
| `read_ts_raw` | raw TS ADC value | — |
| `read_ts_centipercent` | thermistor voltage as % of REGN (0.01 % units) | 0.003 %/LSB |

`read_ts_centipercent` returns `VTS/REGN × 100%` scaled by 100 — for example `5033` represents 50.33%. Pass this result to an NTC lookup table to obtain temperature.

### Register limit helpers

| Method | Returns |
|---|---|
| `read_cell_voltage_limit_mv` | VCELLREG setting converted to mV (offset 3400 mV, 5 mV steps) |

### Charging control helpers

`set_charging_enabled`, `get_charge_status`, `set_recharge_threshold`, `enable_battery_connection`, `is_power_good`.

### Watchdog helpers

`disable_watchdog`, `set_watchdog_timeout`, `reset_watchdog`.

### Interrupt helpers

`mask_all_interrupts`, `read_and_clear_all_flags`.

## Embassy integration

Enable the `embassy` feature to pull in the `bq25887::embassy` module. It provides thin wrappers around `embassy_embedded_hal::shared_bus::asynch::i2c` so you can build a driver from a shared `Mutex`-guarded bus via `new_driver`, or apply per-device bus settings with `new_driver_with_config`. These helpers make it straightforward to share one I²C peripheral across multiple Embassy tasks while keeping charger operations serialized.

```/dev/null/examples/embassy.rs#L1-18
use bq25887::embassy::{new_driver, SharedBus};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

// Replace `MyAsyncI2c` with your HAL's async I²C peripheral.
static BUS: SharedBus<ThreadModeRawMutex, MyAsyncI2c> = Mutex::new(MyAsyncI2c::new());

async fn poll_charger() -> Result<(), bq25887::BQ25887Error<MyAsyncI2c::Error>> {
    let mut driver = new_driver(&BUS);

    let status = driver.read_charger_status_1().await?;
    // Inspect `status` or update application state.
    Ok(())
}
```

---

## Feature flags

| Feature    | Default | Description                                                               |
|------------|---------|---------------------------------------------------------------------------|
| `defmt-03` | ✗       | Enables `defmt` formatting support for logging-friendly targets           |
| `embassy`  | ✗       | Enables Embassy shared-bus helpers via the `bq25887::embassy` module      |
| `log`      | ✗       | Enables `log` crate integration for environments that use `log` backends |

---

## Minimum Supported Rust Version

Rust **1.85** or newer is required. The MSRV will only increase in minor releases and will be documented in the changelog.

---

## Build-time code generation

During `cargo build`, `build.rs` reads `src/bq25887.yaml`, generates the register accessor API, and writes it into `$OUT_DIR`. No manual steps are necessary for downstream users.

---

## Development

```/dev/null/CONTRIBUTING.md#L1-6
cargo fmt --all
cargo clippy --all-targets --all-features
cargo test --all-features
cargo publish --dry-run
```

---

## License

Licensed under either of

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT License](LICENSE-MIT)

at your option.

---

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the project by you shall be dual licensed as above, without any additional terms or conditions.

---

## Links

- 📦 Crate: <https://crates.io/crates/bq25887>
- 📚 Docs: <https://docs.rs/bq25887>
- 🗺️ Repository: <https://github.com/leftger/bq25887>
- 📄 Datasheet: <https://www.ti.com/lit/ug/slusd89b/slusd89b.pdf>