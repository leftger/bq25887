#![no_std]
#![no_main]
#![allow(missing_docs)]

use bq25887::StatusCache;
use bq25887::embassy::{SharedBus, SharedDriverError, new_driver_with_status_cache};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, I2c, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::I2C2;
use embassy_stm32::time::Hertz;
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
    I2C2_EV => i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<I2C2>;
});

type BusMutex = SharedBus<ThreadModeRawMutex, I2c<'static, Async, Master>>;
type DriverError = SharedDriverError<I2c<'static, Async, Master>>;

static BUS: StaticCell<BusMutex> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut config = embassy_stm32::Config::default();

    {
        use embassy_stm32::rcc::*;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,  // PLLM = 1  → HSI / 1 = 16 MHz
            mul: PllMul::MUL25,       // PLLN = 25 → 16 MHz * 25 = 400 MHz VCO
            divr: Some(PllDiv::DIV4), // PLLR = 4  → 100 MHz (Sysclk)
            divq: Some(PllDiv::DIV8), // PLLQ = 8  → 50 MHz
            // divp: Some(PllDiv::DIV25), // PLLP = 25 → 16 MHz (USB_OTG_HS)
            divp: None,
            frac: Some(0), // Fractional part (disabled)
        });

        config.rcc.hse = Some(Hse {
            prescaler: HsePrescaler::DIV1,
        });

        config.rcc.voltage_scale = VoltageScale::RANGE1;
        config.rcc.mux.otghssel = mux::Otghssel::HSE;
        config.rcc.mux.sai1sel = mux::Sai1sel::PLL1_Q;
        config.rcc.sys = Sysclk::PLL1_R;

        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.apb7_pre = APBPrescaler::DIV1;
        config.rcc.ahb5_pre = AHB5Prescaler::DIV4;
    }

    let p = embassy_stm32::init(config);

    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = Hertz(400_000);
    i2c_cfg.gpio_speed = Speed::VeryHigh;
    i2c_cfg.scl_pullup = false;
    i2c_cfg.sda_pullup = false;

    let i2c = I2c::new(p.I2C2, p.PB10, p.PB11, Irqs, p.GPDMA1_CH0, p.GPDMA1_CH1, i2c_cfg);

    let bus = BUS.init(Mutex::new(i2c));
    let _power_on = Output::new(p.PB2, Level::High, Speed::Low);

    spawner
        .spawn(charger_monitor_task(bus))
        .expect("spawn charger_monitor_task");
    spawner.spawn(telemetry_task(bus)).expect("spawn telemetry_task");
    spawner.spawn(control_task(bus)).expect("spawn control_task");

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
                    "[monitor] CHRG_STAT={:?} WD={} VINDPM={} IINDPM={}",
                    defmt::Debug2Format(&status.chrg_stat()),
                    status.wd_stat(),
                    status.vindpm_stat(),
                    status.iindpm_stat()
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

            info!("[control] Ensuring EN_HIZ remains cleared");
            if let Some(mut cached) = driver.configuration_cache().charge_current_limit {
                cached.set_en_hiz(false);
                if let Err(err) = driver.write_charge_current_limit(cached).await {
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

async fn collect_snapshot(bus: &'static BusMutex) -> Result<StatusCache, DriverError> {
    let mut driver = new_driver_with_status_cache(bus).await?;
    driver.refresh_status_register_cache().await?;
    Ok(*driver.status_cache())
}

fn pretty_print_snapshot(snapshot: StatusCache) {
    info!("[telemetry] ---- cached status snapshot ----");

    if let Some(status) = snapshot.charger_status_1 {
        info!(
            "  ChargerStatus1: CHRG_STAT={:?} WD={} VINDPM={} IINDPM={}",
            defmt::Debug2Format(&status.chrg_stat()),
            status.wd_stat(),
            status.vindpm_stat(),
            status.iindpm_stat()
        );
    } else {
        warn!("  ChargerStatus1: <missing>");
    }

    if let Some(faults) = snapshot.fault_status {
        info!(
            "  FaultStatus: VBUS_OVP={} TSHUT={} TIMER={}",
            faults.vbus_ovp_stat(),
            faults.tshut_stat(),
            faults.tmr_stat()
        );
    } else {
        warn!("  FaultStatus: <missing>");
    }

    info!("------------------------------------------");
}

fn log_error(label: &str, err: DriverError) {
    warn!("[error] {}: {:?}", label, defmt::Debug2Format(&err));
}
