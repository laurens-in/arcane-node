use embedded_hal::can::{Id, StandardId};

pub struct ArcaneId(u16);

impl ArcaneId {
    pub fn new(function_code: ArcaneCode, node_id: u8) -> Option<Self> {
        if node_id <= 0x7F {
            Some(ArcaneId(((function_code as u16) << 7) | (node_id as u16)))
        } else {
            None
        }
    }

    pub fn as_raw(&self) -> u16 {
        self.0
    }

    pub fn as_can_id(&self) -> Id {
        Id::Standard(self.as_standard_id())
    }

    pub fn as_standard_id(&self) -> StandardId {
        StandardId::new(self.0).unwrap()
    }
}

pub enum ArcaneCode {
    NMT = 0x0,
    MIDI0 = 0x1,
    RAW0 = 0x2,
    MIDI1 = 0x3,
    RAW1 = 0x4,
    MIDI2 = 0x5,
    RAW2 = 0x6,
    CFGW = 0x7,
    CFGR = 0x8,
}
