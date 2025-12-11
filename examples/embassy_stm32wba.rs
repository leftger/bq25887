#![no_std]
#![no_main]

use core::fmt::Debug;

use bq25887::embassy::{SharedBus, new_driver_with_status_cache};
use bq25887::{BQ25887Error, StatusCache};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::config::Config;
use embassy_stm32::i2c::{self, I2c, NoDma};
use embassy_stm32::peripherals::{I2C1, PB8, PB9};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[cfg(not(feature = "embassy"))]
compile_error!("Enable the `embassy` feature to build this example.");

#[cfg(not(feature = "defmt-03"))]
compile_error!("Enable the `defmt-03` feature to build this example.");

#[cfg(not(target_os = "none"))]
compile_error!("Build this example for a bare-metal target (e.g. thumbv8m.main-none-eabihf).");

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::InterruptHandler<I2C1>;
    I2C1_ER => i2c::InterruptHandler<I2C1>;
});

type BusMutex = SharedBus<ThreadModeRawMutex, I2c<'static, I2C1, NoDma, NoDma>>;

static BUS: StaticCell<BusMutex> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut cfg = Config::default();
    cfg.rcc.hsi = true;
    cfg.rcc.sys_ck = Some(embassy_stm32::time::mhz(96));

    let peripherals = embassy_stm32::init(cfg);

    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = 400_000;

    let i2c = I2c::new(
        peripherals.I2C1,
        peripherals.PB8,
        peripherals.PB9,
        Irqs,
        NoDma,
        NoDma,
        i2c_cfg,
    );

    let bus = BUS.init(Mutex::new(i2c));

    spawner.spawn(charger_monitor_task(bus)).unwrap();
    spawner.spawn(telemetry_task(bus)).unwrap();
    spawner.spawn(control_task(bus)).unwrap();

    loop {
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn charger_monitor_task(bus: &'static BusMutex) {
    match new_driver_with_status_cache(bus).await {
        Ok(mut driver) => loop {
            if let Err(err) = driver.refresh_status_register_cache().await {
                log_error("refresh_status_register_cache", err);
                break;
            }

            if let Some(status) = driver.status_cache().charger_status_1 {
                info!(
                    "[monitor] CHRG_STAT={} VSYS_STAT={} WATCHDOG={}",
                    status.chrg_stat(),
                    status.vsys_stat(),
                    status.watchdog_fault()
                );
            } else {
                warn!("[monitor] charger_status_1 cache empty");
            }

            Timer::after_millis(250).await;
        },
        Err(err) => log_error("charger_monitor_task:new_driver", err),
    }
}

#[embassy_executor::task]
async fn telemetry_task(bus: &'static BusMutex) {
    match collect_snapshot(bus).await {
        Ok(snapshot) => pretty_print_snapshot(snapshot),
        Err(err) => log_error("telemetry_task", err),
    }
}

#[embassy_executor::task]
async fn control_task(bus: &'static BusMutex) {
    match new_driver_with_status_cache(bus).await {
        Ok(mut driver) => {
            if let Err(err) = driver.refresh_configuration_cache().await {
                log_error("refresh_configuration_cache", err);
                return;
            }

            info!("[control] Raising charge-current limit by one step");
            if let Some(mut cached) = driver.configuration_cache().charge_current_limit {
                let updated = cached.with_ichg(cached.ichg().saturating_add(1));
                if let Err(err) = driver.write_charge_current_limit(updated).await {
                    log_error("write_charge_current_limit", err);
                }
            } else {
                warn!("[control] charge_current_limit cache empty, using defaults");
            }

            if let Err(err) = driver.enable_battery_connection(true).await {
                log_error("enable_battery_connection", err);
            }
        }
        Err(err) => log_error("control_task:new_driver", err),
    }
}

async fn collect_snapshot(bus: &'static BusMutex) -> Result<StatusCache, BQ25887Error<i2c::Error>> {
    let mut driver = new_driver_with_status_cache(bus).await?;
    driver.refresh_status_register_cache().await?;
    Ok(*driver.status_cache())
}

fn pretty_print_snapshot(snapshot: StatusCache) {
    info!("[telemetry] ---- cached status snapshot ----");

    if let Some(status) = snapshot.charger_status_1 {
        info!(
            "  ChargerStatus1: CHRG_STAT={} VSYS_STAT={}",
            status.chrg_stat(),
            status.vsys_stat()
        );
    } else {
        warn!("  ChargerStatus1: <missing>");
    }

    if let Some(faults) = snapshot.fault_status {
        info!(
            "  FaultStatus: INPUT={} THERMAL={} TIMER={}",
            faults.input_fault(),
            faults.thermal_shutdown(),
            faults.safety_timer_expired()
        );
    } else {
        warn!("  FaultStatus: <missing>");
    }

    info!("------------------------------------------");
}

fn log_error(label: &str, err: BQ25887Error<i2c::Error>) {
    warn!("[error] {}: {:?}", label, Debug2Format(err));
}

struct Debug2Format<T>(T);

impl<T: Debug> defmt::Format for Debug2Format<T> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{:?}", self.0);
    }
}
