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

```/dev/null/examples/async.rs#L1-48
use bq25887::Bq25887Driver;

pub async fn configure_charger<I2C>(i2c: I2C) -> Result<(), bq25887::BQ25887Error<I2C::Error>>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    let mut driver = Bq25887Driver::new(i2c);

    // Inspect the current regulation limit (register 0x00)
    let limit = driver.read_voltage_regulation_limit().await?;

    // Raise the regulation voltage slightly (example value only!)
    let new_limit = limit.with_vcellreg(limit.vcellreg().saturating_add(1));
    driver.write_voltage_regulation_limit(new_limit).await?;

    // Check charger status after applying the new limit
    let status = driver.read_charger_status_1().await?;
    if status.chrg_stat() != 0 {
        // handle charger state change here (for example, log telemetry or notify a power-management state machine)
    }

    Ok(())
}
```

---

## Charger configuration helpers

The driver exposes high-level methods for each register group, e.g. `read_charge_current_limit`, `write_input_voltage_limit`, and friends. These helpers wrap the generated register accessors and convert conversion errors into the crate’s custom error type.

---

## Feature flags

| Feature    | Default | Description                                                               |
|------------|---------|---------------------------------------------------------------------------|
| `defmt-03` | ✗       | Enables `defmt` formatting support for logging-friendly targets           |
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