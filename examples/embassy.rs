#![allow(clippy::print_stdout)]
#![allow(missing_docs)]

#[cfg(feature = "embassy")]
mod demo {
    use core::convert::Infallible;
    use core::future::{Ready, pending, ready};

    use bq25887::BQ25887Error;
    use bq25887::embassy::{SharedBus, new_driver};
    use embassy_executor::Spawner;
    use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
    use embedded_hal_async::i2c::I2c;
    use futures::future::yield_now;

    const RESET_IMAGE: [u8; 0x20] = [
        0xA0, 0x5E, 0x84, 0x39, 0x41, 0x81, 0x08, 0x00, 0x00, 0x00, 0x00, 0x30, 0x04, 0x10, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ];

    #[derive(Debug, Clone, Copy)]
    struct FakeI2c {
        registers: [u8; 0x20],
    }

    impl FakeI2c {
        const fn new() -> Self {
            Self { registers: RESET_IMAGE }
        }

        fn read_register(&self, address: u8) -> u8 {
            self.registers.get(address as usize).copied().unwrap_or_default()
        }

        fn write_register(&mut self, address: u8, value: u8) {
            if let Some(slot) = self.registers.get_mut(address as usize) {
                *slot = value;
            }
        }
    }

    static BUS: SharedBus<ThreadModeRawMutex, FakeI2c> = SharedBus::new(FakeI2c::new());

    impl I2c for FakeI2c {
        type Error = Infallible;
        type ReadFuture<'a>
            = Ready<Result<(), Self::Error>>
        where
            Self: 'a;
        type WriteFuture<'a>
            = Ready<Result<(), Self::Error>>
        where
            Self: 'a;
        type WriteReadFuture<'a>
            = Ready<Result<(), Self::Error>>
        where
            Self: 'a;

        fn read<'a>(&'a mut self, _address: u8, buffer: &'a mut [u8]) -> Self::ReadFuture<'a> {
            buffer.fill(0);
            ready(Ok(()))
        }

        fn write<'a>(&'a mut self, _address: u8, data: &'a [u8]) -> Self::WriteFuture<'a> {
            if let Some((&register, payload)) = data.split_first() {
                for (offset, byte) in payload.iter().copied().enumerate() {
                    let address = register.wrapping_add(offset as u8);
                    self.write_register(address, byte);
                }
            }
            ready(Ok(()))
        }

        fn write_read<'a>(&'a mut self, _address: u8, tx: &'a [u8], rx: &'a mut [u8]) -> Self::WriteReadFuture<'a> {
            if let Some(&register) = tx.first() {
                for (offset, slot) in rx.iter_mut().enumerate() {
                    let address = register.wrapping_add(offset as u8);
                    *slot = self.read_register(address);
                }
            } else {
                rx.fill(0);
            }
            ready(Ok(()))
        }
    }

    #[embassy_executor::task]
    async fn charger_monitor_task(bus: &'static SharedBus<ThreadModeRawMutex, FakeI2c>) {
        let mut driver = new_driver(bus);
        for poll in 1..=3 {
            match driver.read_charger_status_1().await {
                Ok(status) => {
                    println!(
                        "[monitor #{poll}] CHRG_STAT={}, VSYS_STAT={}, WATCHDOG={}",
                        status.chrg_stat(),
                        status.vsys_stat(),
                        status.watchdog_fault()
                    );
                }
                Err(err) => {
                    log_error("charger_monitor_task: read_charger_status_1", err);
                    break;
                }
            }
            yield_now().await;
        }
    }

    #[embassy_executor::task]
    async fn telemetry_task(bus: &'static SharedBus<ThreadModeRawMutex, FakeI2c>) {
        let mut driver = new_driver(bus);
        match driver.read_charger_status_1().await {
            Ok(status) => {
                println!("[telemetry] ---- status snapshot ----");
                println!(
                    "  ChargerStatus1: CHRG_STAT={}, VSYS_STAT={}",
                    status.chrg_stat(),
                    status.vsys_stat()
                );
            }
            Err(err) => log_error("telemetry_task: read_charger_status_1", err),
        }

        match driver.read_fault_status().await {
            Ok(faults) => {
                println!(
                    "  FaultStatus: INPUT={}, THERMAL={}, TIMER={}",
                    faults.input_fault(),
                    faults.thermal_shutdown(),
                    faults.safety_timer_expired()
                );
            }
            Err(err) => log_error("telemetry_task: read_fault_status", err),
        }
        println!("------------------------------------------");
    }

    #[embassy_executor::task]
    async fn control_task(bus: &'static SharedBus<ThreadModeRawMutex, FakeI2c>) {
        let mut driver = new_driver(bus);

        println!("[control] Reading current charge-current limit");
        match driver.read_charge_current_limit().await {
            Ok(cached) => {
                let new_limit = cached.with_ichg(cached.ichg().saturating_add(1));
                println!("[control] Raising charge-current limit by one step");
                if let Err(err) = driver.write_charge_current_limit(new_limit).await {
                    log_error("control_task: write_charge_current_limit", err);
                    return;
                }
            }
            Err(err) => {
                log_error("control_task: read_charge_current_limit", err);
                return;
            }
        }

        println!("[control] Enabling battery connection");
        if let Err(err) = driver.enable_battery_connection(true).await {
            log_error("control_task: enable_battery_connection", err);
        }
    }

    fn log_error(label: &str, err: BQ25887Error<Infallible>) {
        println!("[error] {label}: {:?}", err);
    }

    pub async fn run(spawner: Spawner) {
        spawner
            .spawn(charger_monitor_task(&BUS))
            .expect("spawn charger_monitor_task");
        spawner.spawn(telemetry_task(&BUS)).expect("spawn telemetry_task");
        spawner.spawn(control_task(&BUS)).expect("spawn control_task");

        pending::<()>().await;
    }
}

#[cfg(feature = "embassy")]
#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    demo::run(spawner).await;
}

#[cfg(not(feature = "embassy"))]
fn main() {
    println!(
        "Enable the `embassy` feature to run this example:\n\
         cargo run --example embassy --features embassy"
    );
}
