pub mod crsf;
pub mod ibus;

pub use crsf::{CrsfParser, CrsfFrame, CRSF_PACKET_SIZE, CRSF_NUM_CHANNELS};
pub use ibus::{IbusParser, IbusFrame, IBUS_PACKET_SIZE, IBUS_NUM_CHANNELS};

pub const MAX_RC_CHANNELS: usize = 16;

/// Unified representation of RC Channel data (in microseconds 1000..2000 us)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcChannels {
    pub channels: [u16; MAX_RC_CHANNELS],
    pub failsafe: bool,
}

impl Default for RcChannels {
    fn default() -> Self {
        Self {
            channels: [1500u16; MAX_RC_CHANNELS],
            failsafe: false,
        }
    }
}

impl From<CrsfFrame> for RcChannels {
    fn from(frame: CrsfFrame) -> Self {
        let mut channels = [1500u16; MAX_RC_CHANNELS];
        channels[..CRSF_NUM_CHANNELS].copy_from_slice(&frame.channels);
        Self {
            channels,
            failsafe: false,
        }
    }
}

impl From<IbusFrame> for RcChannels {
    fn from(frame: IbusFrame) -> Self {
        let mut channels = [1500u16; MAX_RC_CHANNELS];
        channels[..IBUS_NUM_CHANNELS].copy_from_slice(&frame.channels);
        Self {
            channels,
            failsafe: false,
        }
    }
}
