pub const CRSF_PACKET_SIZE: usize = 26;
pub const CRSF_NUM_CHANNELS: usize = 16;
pub const CRSF_SYNC_BYTE: u8 = 0xC8;
pub const CRSF_FRAMETYPE_RC_CHANNELS: u8 = 0x16;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrsfFrame {
    pub channels: [u16; CRSF_NUM_CHANNELS], // Channels in microseconds (988..2012 us)
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum State {
    Sync,
    Length,
    Type,
    Payload,
    Checksum,
}

pub struct CrsfParser {
    buffer: [u8; 64],
    index: usize,
    expected_len: usize,
    state: State,
}

impl Default for CrsfParser {
    fn default() -> Self {
        Self::new()
    }
}

impl CrsfParser {
    pub const fn new() -> Self {
        Self {
            buffer: [0u8; 64],
            index: 0,
            expected_len: 0,
            state: State::Sync,
        }
    }

    pub fn reset(&mut self) {
        self.index = 0;
        self.expected_len = 0;
        self.state = State::Sync;
    }

    /// Process a single byte from UART stream.
    /// Returns Some(CrsfFrame) when a valid RC channel packet is completed.
    pub fn parse_byte(&mut self, byte: u8) -> Option<CrsfFrame> {
        match self.state {
            State::Sync => {
                if byte == CRSF_SYNC_BYTE {
                    self.buffer[0] = byte;
                    self.index = 1;
                    self.state = State::Length;
                }
            }
            State::Length => {
                if (2..=62).contains(&byte) {
                    self.expected_len = byte as usize;
                    self.buffer[1] = byte;
                    self.index = 2;
                    self.state = State::Type;
                } else {
                    self.reset();
                }
            }
            State::Type => {
                self.buffer[self.index] = byte;
                self.index += 1;
                self.state = State::Payload;
            }
            State::Payload => {
                self.buffer[self.index] = byte;
                self.index += 1;
                // expected_len includes type + payload + crc
                if self.index >= self.expected_len + 1 {
                    self.state = State::Checksum;
                }
            }
            State::Checksum => {
                self.buffer[self.index] = byte;
                let crc = calculate_crc8_dvb_s2(&self.buffer[2..self.index]);
                let is_valid = crc == byte;
                let frame_type = self.buffer[2];

                let result = if is_valid
                    && frame_type == CRSF_FRAMETYPE_RC_CHANNELS
                    && self.index == CRSF_PACKET_SIZE - 1
                {
                    Some(decode_rc_channels(&self.buffer[3..self.index]))
                } else {
                    None
                };

                self.reset();
                return result;
            }
        }
        None
    }
}

pub fn calculate_crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc = 0x00u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

fn decode_rc_channels(payload: &[u8]) -> CrsfFrame {
    let mut channels = [1500u16; CRSF_NUM_CHANNELS];
    let mut bits_merged = 0usize;
    let mut read_val = 0u32;
    let mut byte_idx = 0usize;

    for ch in channels.iter_mut() {
        while bits_merged < 11 && byte_idx < payload.len() {
            read_val |= (payload[byte_idx] as u32) << bits_merged;
            bits_merged += 8;
            byte_idx += 1;
        }
        let ticks = (read_val & 0x07FF) as u16;
        read_val >>= 11;
        bits_merged -= 11;
        // Convert CRSF ticks (172..1811) to us (988..2012)
        *ch = (ticks as u32 * 5 / 8 + 880) as u16;
    }

    CrsfFrame { channels }
}
