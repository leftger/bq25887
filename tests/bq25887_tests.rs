#![allow(missing_docs)]

use core::convert::TryFrom;

use bq25887::field_sets::{ChargeCurrentLimit, FieldSetValue, PartInformation};
use bq25887::{Bq25887, Ichg, PartInformationSummary, Pn};
use device_driver::RegisterInterface;

struct DummyInterface;

impl RegisterInterface for DummyInterface {
    type AddressType = u8;
    type Error = ();

    fn read_register(&mut self, _addr: u8, _offset: u32, buffer: &mut [u8]) -> Result<(), Self::Error> {
        buffer.fill(0xDE);
        Ok(())
    }

    fn write_register(&mut self, _addr: u8, _offset: u32, _data: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn test_read_reg01() {
    let mut dev = Bq25887::new(DummyInterface);
    let reg01 = dev.charge_current_limit().read().unwrap();
    assert!(reg01.en_hiz());
    assert!(reg01.en_ilim());
}

#[test]
fn test_read_all_registers() {
    let mut dev = Bq25887::new(DummyInterface);
    let mut count = 0;
    dev.read_all_registers(|addr, name, _val| {
        println!("Read register {name} (0x{addr:02X})");
        count += 1;
    })
    .unwrap();
    assert_eq!(count, 45);
}

#[test]
fn test_ichg_enum_roundtrip() {
    let raw = 0x1D;
    let enum_val = Ichg::try_from(raw).unwrap();
    assert_eq!(u8::from(enum_val), raw);
}

#[test]
fn test_reg01_parsing_from_raw() {
    let reg = ChargeCurrentLimit::from([0xDE]); // 0b1101_1110
    assert!(reg.en_hiz());
    assert!(reg.en_ilim());
    assert_eq!(reg.ichg().unwrap(), Ichg::MA1550);
}

#[test]
fn test_reg01_building() {
    let mut reg = ChargeCurrentLimit::new_zero();
    reg.set_en_hiz(true);
    reg.set_en_ilim(true);
    reg.set_ichg(Ichg::MA1550);
    assert_eq!(<[u8; 1]>::from(reg), [0xDE]);
}

#[test]
fn test_read_all_registers_executes() {
    struct ZeroInterface;

    impl RegisterInterface for ZeroInterface {
        type AddressType = u8;
        type Error = core::convert::Infallible;

        fn read_register(&mut self, _addr: u8, _offset: u32, buffer: &mut [u8]) -> Result<(), Self::Error> {
            buffer.fill(0x00);
            Ok(())
        }

        fn write_register(&mut self, _addr: u8, _offset: u32, _data: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    let mut dev = Bq25887::new(ZeroInterface);
    let mut count = 0;
    dev.read_all_registers(|_addr, _name, _val: FieldSetValue| {
        count += 1;
    })
    .unwrap();
    assert_eq!(count, 45);
}

#[test]
fn test_part_information_summary_success() {
    let reg = PartInformation::from([0x28]);
    let summary = PartInformationSummary::try_from(reg).unwrap();

    assert_eq!(summary.part_number, Pn::Bq25887);
    assert_eq!(summary.device_revision, 0);
    assert!(!summary.register_reset);
}

#[test]
fn test_part_information_summary_register_reset() {
    let reg = PartInformation::from([0b1010_1000]);
    let summary = PartInformationSummary::try_from(reg).unwrap();

    assert_eq!(summary.part_number, Pn::Bq25887);
    assert_eq!(summary.device_revision, 0);
    assert!(summary.register_reset);
}

#[test]
fn test_part_information_summary_invalid_pn() {
    let reg = PartInformation::from([0b0110_0001]);
    let result = PartInformationSummary::try_from(reg);
    assert!(result.is_err());
}

#[test]
fn test_reg01_defaults() {
    let reg = ChargeCurrentLimit::new();
    assert_eq!(<[u8; 1]>::from(reg), [0x5E]);
}
