use crate::math::map_range;

pub const DSHOT_MIN_THROTTLE: u16 = 48;
pub const DSHOT_MAX_THROTTLE: u16 = 2047;

const DSHOT_ZERO_PATTERN: u8 = 0b11100000;
const DSHOT_ONE_PATTERN: u8 = 0b11111100;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DshotSpeed {
    Dshot150, // 1.2 MHz SPI
    Dshot300, // 2.4 MHz SPI
    Dshot600, // 4.8 MHz SPI
}

pub struct DshotEncoder;

impl DshotEncoder {
    /// Encodes a 16-bit DShot packet with CRC
    pub fn encode_packet(throttle: u16, telemetry: bool) -> u16 {
        let mut t = throttle;
        if t > 0 && t < DSHOT_MIN_THROTTLE {
            t = DSHOT_MIN_THROTTLE;
        } else if t > DSHOT_MAX_THROTTLE {
            t = DSHOT_MAX_THROTTLE;
        }

        let mut data = t << 1;
        if telemetry {
            data |= 1;
        }

        let checksum = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
        (data << 4) | checksum
    }

    /// Converts a 16-bit packet into 16 SPI MOSI bytes for bitbanging
    pub fn to_spi_buffer(packet: u16, buffer: &mut [u8; 16]) {
        for i in 0..16 {
            let bit = (packet >> (15 - i)) & 0x01;
            buffer[i] = if bit == 1 {
                DSHOT_ONE_PATTERN
            } else {
                DSHOT_ZERO_PATTERN
            };
        }
    }

    /// Maps standard 1000..2000 us RC pulse to DShot 0..2047
    pub fn map_rc_pulse_to_dshot(pulse_us: u16) -> u16 {
        if pulse_us < 1050 {
            0
        } else {
            map_range(pulse_us as f32, 1000.0, 2000.0, 48.0, 2047.0) as u16
        }
    }
}
