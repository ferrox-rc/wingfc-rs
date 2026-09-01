pub const IBUS_PACKET_SIZE: usize = 32;
pub const IBUS_NUM_CHANNELS: usize = 14;
pub const IBUS_SYNC_BYTE1: u8 = 0x20;
pub const IBUS_SYNC_BYTE2: u8 = 0x40;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IbusFrame {
    pub channels: [u16; IBUS_NUM_CHANNELS],
}

pub struct IbusParser {
    buffer: [u8; IBUS_PACKET_SIZE],
    index: usize,
}

impl Default for IbusParser {
    fn default() -> Self {
        Self::new()
    }
}

impl IbusParser {
    pub const fn new() -> Self {
        Self {
            buffer: [0u8; IBUS_PACKET_SIZE],
            index: 0,
        }
    }

    pub fn parse_byte(&mut self, byte: u8) -> Option<IbusFrame> {
        if self.index == 0 {
            if byte == IBUS_SYNC_BYTE1 {
                self.buffer[0] = byte;
                self.index = 1;
            }
            return None;
        }

        if self.index == 1 {
            if byte == IBUS_SYNC_BYTE2 {
                self.buffer[1] = byte;
                self.index = 2;
            } else {
                self.index = 0;
            }
            return None;
        }

        self.buffer[self.index] = byte;
        self.index += 1;

        if self.index == IBUS_PACKET_SIZE {
            self.index = 0;
            let mut sum: u32 = 0;
            for &b in &self.buffer[0..30] {
                sum += b as u32;
            }
            let calculated_chk = (0xFFFF - (sum & 0xFFFF)) as u16;
            let packet_chk = (self.buffer[30] as u16) | ((self.buffer[31] as u16) << 8);

            if calculated_chk == packet_chk {
                let mut channels = [1500u16; IBUS_NUM_CHANNELS];
                for (i, ch) in channels.iter_mut().enumerate() {
                    let offset = 2 + i * 2;
                    *ch = (self.buffer[offset] as u16) | ((self.buffer[offset + 1] as u16) << 8);
                }
                return Some(IbusFrame { channels });
            }
        }
        None
    }
}
