pub const IBUS_PACKET_SIZE: usize = 32;
pub const IBUS_NUM_CHANNELS: usize = 14;
pub const IBUS_HEADER1: u8 = 0x20; // 32-byte packet length header
pub const IBUS_HEADER2: u8 = 0x40; // FlySky command byte
pub const IBUS_BAUD_RATE: u32 = 115200;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IbusFrame {
    pub channels: [u16; IBUS_NUM_CHANNELS], // Channel values in microseconds (typically 1000..2000 us)
}

impl Default for IbusFrame {
    fn default() -> Self {
        Self {
            channels: [1500u16; IBUS_NUM_CHANNELS],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum IbusState {
    WaitingForHeader1,
    WaitingForHeader2,
    ReadingPayload,
}

pub struct IbusParser {
    buffer: [u8; IBUS_PACKET_SIZE],
    index: usize,
    state: IbusState,
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
            state: IbusState::WaitingForHeader1,
        }
    }

    pub fn reset(&mut self) {
        self.index = 0;
        self.state = IbusState::WaitingForHeader1;
    }

    /// Process a single byte from the UART receiver stream.
    /// Returns Some(IbusFrame) when a complete and valid frame is verified.
    pub fn parse_byte(&mut self, byte: u8) -> Option<IbusFrame> {
        match self.state {
            IbusState::WaitingForHeader1 => {
                if byte == IBUS_HEADER1 {
                    self.buffer[0] = byte;
                    self.index = 1;
                    self.state = IbusState::WaitingForHeader2;
                }
            }
            IbusState::WaitingForHeader2 => {
                if byte == IBUS_HEADER2 {
                    self.buffer[1] = byte;
                    self.index = 2;
                    self.state = IbusState::ReadingPayload;
                } else {
                    self.reset();
                }
            }
            IbusState::ReadingPayload => {
                self.buffer[self.index] = byte;
                self.index += 1;

                if self.index >= IBUS_PACKET_SIZE {
                    let frame = if self.verify_checksum() {
                        Some(self.decode_channels())
                    } else {
                        None
                    };

                    self.reset();
                    return frame;
                }
            }
        }
        None
    }

    /// Calculates 16-bit FlySky checksum: 0xFFFF - sum(bytes[0..30])
    #[inline]
    fn verify_checksum(&self) -> bool {
        let mut calculated_checksum: u16 = 0xFFFF;
        for i in 0..(IBUS_PACKET_SIZE - 2) {
            calculated_checksum = calculated_checksum.wrapping_sub(self.buffer[i] as u16);
        }

        let received_checksum = (self.buffer[IBUS_PACKET_SIZE - 2] as u16)
            | ((self.buffer[IBUS_PACKET_SIZE - 1] as u16) << 8);

        calculated_checksum == received_checksum
    }

    #[inline]
    fn decode_channels(&self) -> IbusFrame {
        let mut channels = [1500u16; IBUS_NUM_CHANNELS];
        for (i, ch) in channels.iter_mut().enumerate() {
            let offset = 2 + i * 2;
            let val = (self.buffer[offset] as u16) | ((self.buffer[offset + 1] as u16) << 8);
            *ch = val;
        }
        IbusFrame { channels }
    }
}
