use modular_bitfield::prelude::*;

pub enum RegisterAddr {
    ChargeCurrentLimit,
    InputVoltageLimit,
    InputCurrentLimit,
    PrechargeAndTerminationCurrentLimit,
    ChargerControl1,
    ChargerControl2,
    ChargerControl3,
    ChargerControl4,
    ChargerMask1,
    ChargerMask2,
    FaultMask,
    AdcControl,
    AdcFunctionDisable,
    PartInformation,
    CellBalancingControl1,
    CellBalancingControl2,
    CellBalancingStatusAndControl,
    CellBalancingFlag,
    CellBalancingMask,
}

impl RegisterAddr {
    pub fn value(&self) -> u8 {
        match self {
            RegisterAddr::ChargeCurrentLimit => 0x01,
            RegisterAddr::InputVoltageLimit => 0x02,
            RegisterAddr::InputCurrentLimit => 0x03,
            RegisterAddr::PrechargeAndTerminationCurrentLimit => 0x04,
            RegisterAddr::ChargerControl1 => 0x05,
            RegisterAddr::ChargerControl2 => 0x06,
            RegisterAddr::ChargerControl3 => 0x07,
            RegisterAddr::ChargerControl4 => 0x08,
            RegisterAddr::ChargerMask1 => 0x12,
            RegisterAddr::ChargerMask2 => 0x13,
            RegisterAddr::FaultMask => 0x14,
            RegisterAddr::AdcControl => 0x15,
            RegisterAddr::AdcFunctionDisable => 0x16,
            RegisterAddr::PartInformation => 0x25,
            RegisterAddr::CellBalancingControl1 => 0x28,
            RegisterAddr::CellBalancingControl2 => 0x29,
            RegisterAddr::CellBalancingStatusAndControl => 0x2A,
            RegisterAddr::CellBalancingFlag => 0x2B,
            RegisterAddr::CellBalancingMask => 0x2C,
        }
    }
}

#[bitfield]
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VoltageRegulationLimit {
    pub vcellreg: B8,
}

impl From<VoltageRegulationLimit> for u8 {
    fn from(value: VoltageRegulationLimit) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargeCurrentLimit {
    pub ichg: B6,
    pub en_ilim: B1,
    pub en_hiz: B1,
}

impl From<ChargeCurrentLimit> for u8 {
    fn from(value: ChargeCurrentLimit) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct InputVoltageLimit {
    pub vindpm: B5,
    pub pfm_ooa_dis: B1,
    pub en_bat_dischg: B1,
    pub en_vindpm_rst: B1,
}

impl From<InputVoltageLimit> for u8 {
    fn from(value: InputVoltageLimit) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct InputCurrentLimit {
    pub iindpm: B5,
    pub en_ico: B1,
    pub force_indet: B1,
    pub force_ico: B1,
}

impl From<InputCurrentLimit> for u8 {
    fn from(value: InputCurrentLimit) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct PrechargeAndTerminationCurrentLimit {
    pub iterm: B4,
    pub iprechg: B4,
}

impl From<PrechargeAndTerminationCurrentLimit> for u8 {
    fn from(value: PrechargeAndTerminationCurrentLimit) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerControl1 {
    pub tmr2x_en: B1,
    pub chg_timer: B2,
    pub en_timer: B1,
    pub watchdog: B2,
    pub stat_dis: B1,
    pub en_term: B1,
}

impl From<ChargerControl1> for u8 {
    fn from(value: ChargerControl1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerControl2 {
    pub vrechg: B2,
    pub celllowv: B1,
    pub en_chg: B1,
    pub treg: B2,
    pub auto_indet_en: B1,
    #[skip(setters, getters)]
    reseved: B1,
}

impl From<ChargerControl2> for u8 {
    fn from(value: ChargerControl2) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerControl3 {
    #[skip(setters, getters)]
    reserved: B4,
    pub topoff_timer: B2,
    pub wd_rst: B1,
    pub pfm_dis: B1,
}

impl From<ChargerControl3> for u8 {
    fn from(value: ChargerControl3) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerControl4 {
    pub jeita_isetc: B2,
    pub jeita_iseth: B1,
    pub jeita_vset: B2,
    #[skip(setters, getters)]
    reserved: B3,
}

impl From<ChargerControl4> for u8 {
    fn from(value: ChargerControl4) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct IcoCurrentLimitInUse {
    pub ico_ilim: B5,
    #[skip(setters, getters)]
    reserved: B3,
}

impl From<IcoCurrentLimitInUse> for u8 {
    fn from(value: IcoCurrentLimitInUse) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerStatus1 {
    pub chrg_stat: B3,
    pub wd_stat: B1,
    pub treg_stat: B1,
    pub vindpm_stat: B1,
    pub iindpm_stat: B1,
    #[skip(setters, getters)]
    reserved: B1,
}

impl From<ChargerStatus1> for u8 {
    fn from(value: ChargerStatus1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerStatus2 {
    #[skip(setters, getters)]
    reserved_1: B1,
    pub ico_stat: B2,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub vbus_stat: B3,
    pub pg_stat: B1,
}

impl From<ChargerStatus2> for u8 {
    fn from(value: ChargerStatus2) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct NTCStatus {
    pub ts_stat: B3,
    #[skip(setters, getters)]
    reserved: B5,
}

impl From<NTCStatus> for u8 {
    fn from(value: NTCStatus) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct FaultStatus {
    #[skip(setters, getters)]
    reserved_1: B1,
    #[skip(setters, getters)]
    reserved_2: B3,
    pub tmr_stat: B1,
    #[skip(setters, getters)]
    reserved_3: B1,
    pub tshut_stat: B1,
    pub vbus_ovp_stat: B1,
}

impl From<FaultStatus> for u8 {
    fn from(value: FaultStatus) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerFlag1 {
    pub chrg_flag: B1,
    #[skip(setters, getters)]
    reserved_1: B2,
    pub wd_flag: B1,
    pub treg_flag: B1,
    pub vindpm_flag: B1,
    pub iindpm_flag: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
}

impl From<ChargerFlag1> for u8 {
    fn from(value: ChargerFlag1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerFlag2 {
    #[skip(setters, getters)]
    reserved_1: B1,
    pub ico_flag: B1,
    pub ts_flag: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub vbus_flag: B1,
    #[skip(setters, getters)]
    reserved_3: B2,
    pub pg_flag: B1,
}

impl From<ChargerFlag2> for u8 {
    fn from(value: ChargerFlag2) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct FaultFlag {
    #[skip(setters, getters)]
    reserved_1: B4,
    pub tmr_flag: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub tshut_flag: B1,
    pub vbus_ovp_flag: B1,
}

impl From<FaultFlag> for u8 {
    fn from(value: FaultFlag) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerMask1 {
    pub chrg_mask: B1,
    #[skip(setters, getters)]
    reserved: B2,
    pub wd_mask: B1,
    pub treg_mask: B1,
    pub vinpdm_mask: B1,
    pub iindpm_mask: B1,
    pub adc_done_mask: B1,
}

impl From<ChargerMask1> for u8 {
    fn from(value: ChargerMask1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChargerMask2 {
    #[skip(setters, getters)]
    reserved_1: B1,
    pub ico_mask: B1,
    pub ts_mask: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub vbus_mask: B1,
    #[skip(setters, getters)]
    reserved_3: B2,
    pub pg_mask: B1,
}

impl From<ChargerMask2> for u8 {
    fn from(value: ChargerMask2) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct FaultMask {
    #[skip(setters, getters)]
    reserved_1: B3,
    pub sns_short_mask: B1,
    pub tmr_mask: B1,
    #[skip(setters, getters)]
    reserved_2: B1,
    pub tshut_mask: B1,
    pub vbus_ovp_mask: B1,
}

impl From<FaultMask> for u8 {
    fn from(value: FaultMask) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AdcControl {
    #[skip(setters, getters)]
    reserved: B4,
    pub adc_sample: B2,
    pub adc_rate: B1,
    pub adc_en: B1,
}

impl From<AdcControl> for u8 {
    fn from(value: AdcControl) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AdcFunctionDisable {
    pub tdie_adc_dis: B1,
    pub vcell_adc_dis: B1,
    pub ts_adc_dis: B1,
    #[skip(setters, getters)]
    reserved: B1,
    pub vbat_adc_dis: B1,
    pub vbus_adc_dis: B1,
    pub ichg_adc_dis: B1,
    pub ibus_adc_dis: B1,
}

impl From<AdcFunctionDisable> for u8 {
    fn from(value: AdcFunctionDisable) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct IbusAdc1 {
    pub ibus_adc_msb: B8,
}

impl From<IbusAdc1> for u8 {
    fn from(value: IbusAdc1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct IbusAdc0 {
    pub ibus_adc_lsb: B8,
}

impl From<IbusAdc0> for u8 {
    fn from(value: IbusAdc0) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct IChgAdc1 {
    pub ichg_adc_msb: B8,
}

impl From<IChgAdc1> for u8 {
    fn from(value: IChgAdc1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct IChgAdc0 {
    pub ichg_adc_lsb: B8,
}

impl From<IChgAdc0> for u8 {
    fn from(value: IChgAdc0) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VBusAdc1 {
    pub vbus_adc_msb: B8,
}

impl From<VBusAdc1> for u8 {
    fn from(value: VBusAdc1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VBusAdc0 {
    pub vbus_adc_lsb: B8,
}

impl From<VBusAdc0> for u8 {
    fn from(value: VBusAdc0) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VBatAdc1 {
    pub vbat_adc_msb: B8,
}

impl From<VBatAdc1> for u8 {
    fn from(value: VBatAdc1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VBatAdc0 {
    pub vbat_adc_lsb: B8,
}

impl From<VBatAdc0> for u8 {
    fn from(value: VBatAdc0) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VCellTopAdc1 {
    pub vcelltop_adc_msb: B8,
}

impl From<VCellTopAdc1> for u8 {
    fn from(value: VCellTopAdc1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VCellTopAdc0 {
    pub vcelltop_adc_lsb: B8,
}

impl From<VCellTopAdc0> for u8 {
    fn from(value: VCellTopAdc0) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct TsAdc1 {
    pub ts_adc_msb: B8,
}

impl From<TsAdc1> for u8 {
    fn from(value: TsAdc1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct TsAdc0 {
    pub ts_adc_lsb: B8,
}

impl From<TsAdc0> for u8 {
    fn from(value: TsAdc0) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct TdieAdc1 {
    pub tdie_adc_msb: B8,
}

impl From<TdieAdc1> for u8 {
    fn from(value: TdieAdc1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct TdieAdc0 {
    pub tdie_adc_lsb: B8,
}

impl From<TdieAdc0> for u8 {
    fn from(value: TdieAdc0) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct PartInformation {
    pub dev_rev: B3,
    pub pn: B4,
    pub reg_rst: B1,
}

impl From<PartInformation> for u8 {
    fn from(value: PartInformation) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VCellBotAdc1 {
    pub vcellbot_adc_msb: B8,
}

impl From<VCellBotAdc1> for u8 {
    fn from(value: VCellBotAdc1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct VCellBotAdc0 {
    pub vcellbot_adc_lsb: B8,
}

impl From<VCellBotAdc0> for u8 {
    fn from(value: VCellBotAdc0) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CellBalancingControl1 {
    pub tsettle: B2,
    pub tcb_active: B2,
    pub tcb_qual_interval: B1,
    pub vdiff_end_offset: B3,
}

impl From<CellBalancingControl1> for u8 {
    fn from(value: CellBalancingControl1) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CellBalancingControl2 {
    pub vdiff_start: B4,
    pub vqual_th: B4,
}

impl From<CellBalancingControl2> for u8 {
    fn from(value: CellBalancingControl2) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CellBalancingStatusAndControl {
    pub cb_oc_stat: B1,
    pub ls_ov_stat: B1,
    pub hs_ov_stat: B1,
    pub ls_cv_stat: B1,
    pub hs_cv_stat: B1,
    pub cb_stat: B1,
    pub cb_auto_en: B1,
    pub cb_chg_dis: B1,
}

impl From<CellBalancingStatusAndControl> for u8 {
    fn from(value: CellBalancingStatusAndControl) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CellBalancingFlag {
    pub cb_oc_flag: B1,
    pub ls_ov_flag: B1,
    pub hs_ov_flag: B1,
    pub ls_cv_flag: B1,
    pub hs_cv_flag: B1,
    pub cb_flag: B1,
    pub qcbl_en: B1,
    pub qcbh_en: B1,
}

impl From<CellBalancingFlag> for u8 {
    fn from(value: CellBalancingFlag) -> Self {
        value.bytes[0]
    }
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CellBalancingMask {
    pub cb_oc_mask: B1,
    pub ls_ov_mask: B1,
    pub hs_ov_mask: B1,
    pub ls_cv_mask: B1,
    pub hs_cv_mask: B1,
    pub cb_mask: B1,
    #[skip(setters, getters)]
    reserved: B2,
}

impl From<CellBalancingMask> for u8 {
    fn from(value: CellBalancingMask) -> Self {
        value.bytes[0]
    }
}
