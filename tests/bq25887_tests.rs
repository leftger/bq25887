#![allow(missing_docs)]
#[cfg(test)]
mod tests {
    use bq25887::Bq25887;
    use device_driver::RegisterInterface;

    struct DummyInterface;

    #[derive(Debug)]
    enum DummyError {}

    impl RegisterInterface for DummyInterface {
        type AddressType = u8;
        type Error = DummyError;

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
        assert_eq!(reg01.en_hiz(), true);
        assert_eq!(reg01.en_ilim(), true);
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
        use bq25887::Ichg;
        let raw = 0x1D;
        let enum_val = Ichg::try_from(raw).unwrap();
        assert_eq!(u8::from(enum_val), raw);
    }

    #[test]
    fn test_reg01_parsing_from_raw() {
        use bq25887::field_sets::ChargeCurrentLimit;

        let reg = ChargeCurrentLimit::from([0xDE]); // 0b11011110
        assert!(reg.en_hiz());
        assert!(reg.en_ilim());
        assert_eq!(reg.ichg().unwrap(), bq25887::Ichg::MA1550);
    }

    #[test]
    fn test_reg01_building() {
        use bq25887::field_sets::ChargeCurrentLimit;

        let mut reg = ChargeCurrentLimit::new_zero();
        reg.set_en_hiz(true);
        reg.set_en_ilim(true);
        reg.set_ichg(bq25887::Ichg::MA1550);
        assert_eq!(<[u8; 1]>::from(reg), [0xDE]);
    }

    #[test]
    fn test_read_all_registers_executes() {
        use bq25887::Bq25887;
        use bq25887::field_sets::FieldSetValue;
        use device_driver::RegisterInterface;

        struct Dummy;
        impl RegisterInterface for Dummy {
            type AddressType = u8;
            type Error = core::convert::Infallible;

            fn read_register(&mut self, _addr: u8, _offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
                buf.fill(0x00);
                Ok(())
            }

            fn write_register(&mut self, _addr: u8, _offset: u32, _data: &[u8]) -> Result<(), Self::Error> {
                Ok(())
            }
        }

        let mut dev = Bq25887::new(Dummy);
        let mut count = 0;
        dev.read_all_registers(|_addr, _name, _val: FieldSetValue| {
            count += 1;
        })
        .unwrap();
        assert_eq!(count, 45);
    }

    #[test]
    fn test_reg01_defaults() {
        let reg = bq25887::field_sets::ChargeCurrentLimit::new();
        assert_eq!(<[u8; 1]>::from(reg), [0x5E]);
    }
}
