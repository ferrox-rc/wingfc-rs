pub mod crsf;
pub mod ibus;

pub use crsf::{CrsfParser, CrsfFrame, CRSF_PACKET_SIZE, CRSF_NUM_CHANNELS};
pub use ibus::{IbusParser, IBUS_PACKET_SIZE, IBUS_NUM_CHANNELS};
