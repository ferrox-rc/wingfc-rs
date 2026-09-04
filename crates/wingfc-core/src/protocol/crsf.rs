pub const CRSF_PACKET_SIZE: usize = 26;
pub const CRSF_NUM_CHANNELS: usize = 16;
pub const CRSF_SYNC_BYTE: u8 = 0xC8;

// CRSF Device Addresses
pub const CRSF_ADDRESS_BROADCAST: u8 = 0x00;
pub const CRSF_ADDRESS_FLIGHT_CONTROLLER: u8 = 0xC8;
pub const CRSF_ADDRESS_CRSF_RECEIVER: u8 = 0xEC;
pub const CRSF_ADDRESS_RADIO_TRANSMITTER: u8 = 0xEA;

// CRSF Frame Types
pub const CRSF_FRAMETYPE_GPS: u8 = 0x02;
pub const CRSF_FRAMETYPE_BATTERY_SENSOR: u8 = 0x08;
pub const CRSF_FRAMETYPE_BARO_ALTITUDE: u8 = 0x09;
pub const CRSF_FRAMETYPE_RC_CHANNELS: u8 = 0x16;
pub const CRSF_FRAMETYPE_ATTITUDE: u8 = 0x1E;
pub const CRSF_FRAMETYPE_FLIGHT_MODE: u8 = 0x21;

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
                self.buffer[self.index] = byte;
                self.expected_len = byte as usize;
                if !(3..=62).contains(&self.expected_len) {
                    self.reset();
                } else {
                    self.index += 1;
                    self.state = State::Type;
                }
            }
            State::Type => {
                self.buffer[self.index] = byte;
                self.index += 1;
                if self.index > self.expected_len {
                    self.state = State::Checksum;
                } else {
                    self.state = State::Payload;
                }
            }
            State::Payload => {
                self.buffer[self.index] = byte;
                self.index += 1;
                if self.index > self.expected_len {
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

/// Serializes flight controller sensor and status data into standard Crossfire / ELRS telemetry frames.
pub struct CrsfTelemetryEncoder;

impl CrsfTelemetryEncoder {
    /// Encodes a generic CRSF telemetry frame:
    /// `[SYNC 0xC8] [LEN: payload_len + 2] [FRAME_TYPE] [PAYLOAD...] [CRC8]`
    ///
    /// The CRC-8 covers `[FRAME_TYPE]` and `[PAYLOAD...]`.
    /// Returns the total bytes written into `out` (`payload.len() + 4`), or 0 if `out` is too small.
    pub fn encode_frame(frame_type: u8, payload: &[u8], out: &mut [u8]) -> usize {
        let payload_len = payload.len();
        let total_len = payload_len + 4;
        if out.len() < total_len {
            return 0;
        }
        out[0] = CRSF_SYNC_BYTE;
        out[1] = (payload_len + 2) as u8;
        out[2] = frame_type;
        out[3..3 + payload_len].copy_from_slice(payload);
        let crc = calculate_crc8_dvb_s2(&out[2..3 + payload_len]);
        out[3 + payload_len] = crc;
        total_len
    }

    /// Encodes Flight Mode frame (0x21).
    ///
    /// Payload is a null-terminated ASCII string (e.g. "ACRO\0", "ANGLE\0", "DISARM\0").
    pub fn encode_flight_mode(mode_str: &str, out: &mut [u8]) -> usize {
        let str_bytes = mode_str.as_bytes();
        let mut payload = [0u8; 16];
        let copy_len = str_bytes.len().min(15);
        payload[..copy_len].copy_from_slice(&str_bytes[..copy_len]);
        payload[copy_len] = 0; // null terminator
        Self::encode_frame(CRSF_FRAMETYPE_FLIGHT_MODE, &payload[..copy_len + 1], out)
    }

    /// Encodes Vehicle Attitude frame (0x1E).
    ///
    /// * `pitch_rad`: Pitch angle in radians (stored as `i16` big-endian, radians &times; 10,000)
    /// * `roll_rad`: Roll angle in radians (stored as `i16` big-endian, radians &times; 10,000)
    /// * `yaw_rad`: Yaw angle in radians (stored as `i16` big-endian, radians &times; 10,000)
    pub fn encode_attitude(pitch_rad: f32, roll_rad: f32, yaw_rad: f32, out: &mut [u8]) -> usize {
        let pitch = (pitch_rad * 10000.0) as i16;
        let roll = (roll_rad * 10000.0) as i16;
        let yaw = (yaw_rad * 10000.0) as i16;

        let mut payload = [0u8; 6];
        payload[0..2].copy_from_slice(&pitch.to_be_bytes());
        payload[2..4].copy_from_slice(&roll.to_be_bytes());
        payload[4..6].copy_from_slice(&yaw.to_be_bytes());

        Self::encode_frame(CRSF_FRAMETYPE_ATTITUDE, &payload, out)
    }

    /// Encodes Battery Sensor frame (0x08).
    ///
    /// * `voltage_dv`: Voltage in decivolts (0.1 V resolution, e.g. 11.1 V = 111)
    /// * `current_da`: Current in deciapps (0.1 A resolution, e.g. 5.2 A = 52)
    /// * `capacity_mah`: Consumed battery capacity in mAh (24-bit big-endian)
    /// * `remaining_pct`: Remaining battery percentage (0..100)
    pub fn encode_battery(
        voltage_dv: u16,
        current_da: u16,
        capacity_mah: u32,
        remaining_pct: u8,
        out: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 8];
        payload[0..2].copy_from_slice(&voltage_dv.to_be_bytes());
        payload[2..4].copy_from_slice(&current_da.to_be_bytes());
        payload[4] = (capacity_mah >> 16) as u8;
        payload[5] = (capacity_mah >> 8) as u8;
        payload[6] = capacity_mah as u8;
        payload[7] = remaining_pct;

        Self::encode_frame(CRSF_FRAMETYPE_BATTERY_SENSOR, &payload, out)
    }

    /// Encodes GNSS / GPS Navigation frame (0x02).
    ///
    /// * `lat_deg`: Latitude in decimal degrees (-90.0 .. +90.0) -> scaled by 1e7 (`i32` BE)
    /// * `lon_deg`: Longitude in decimal degrees (-180.0 .. +180.0) -> scaled by 1e7 (`i32` BE)
    /// * `ground_speed_mps`: Speed over ground in m/s -> converted to (km/h &times; 10)
    /// * `heading_deg`: Ground course in degrees (0.0 .. 360.0) -> scaled by 100 (`u16` BE)
    /// * `alt_msl_m`: Altitude MSL in meters -> stored with 1000m offset (`u16` BE)
    /// * `satellites`: Number of tracked satellites
    pub fn encode_gps(
        lat_deg: f64,
        lon_deg: f64,
        ground_speed_mps: f32,
        heading_deg: f32,
        alt_msl_m: f32,
        satellites: u8,
        out: &mut [u8],
    ) -> usize {
        let lat = (lat_deg * 10_000_000.0) as i32;
        let lon = (lon_deg * 10_000_000.0) as i32;
        let speed_kmh_x10 = (ground_speed_mps * 36.0) as u16;
        let heading_cdeg = (heading_deg * 100.0) as u16;
        let altitude = (alt_msl_m + 1000.0).max(0.0) as u16;

        let mut payload = [0u8; 15];
        payload[0..4].copy_from_slice(&lat.to_be_bytes());
        payload[4..8].copy_from_slice(&lon.to_be_bytes());
        payload[8..10].copy_from_slice(&speed_kmh_x10.to_be_bytes());
        payload[10..12].copy_from_slice(&heading_cdeg.to_be_bytes());
        payload[12..14].copy_from_slice(&altitude.to_be_bytes());
        payload[14] = satellites;

        Self::encode_frame(CRSF_FRAMETYPE_GPS, &payload, out)
    }

    /// Encodes Barometer Altitude & Variometer frame (0x09).
    ///
    /// * `alt_msl_m`: Altitude MSL in meters -> stored with 10,000m offset (`u16` BE)
    /// * `vario_cms`: Vertical climb rate in cm/s (`i16` BE)
    pub fn encode_baro_altitude(alt_msl_m: f32, vario_cms: i16, out: &mut [u8]) -> usize {
        let altitude = (alt_msl_m + 10000.0).max(0.0) as u16;
        let mut payload = [0u8; 4];
        payload[0..2].copy_from_slice(&altitude.to_be_bytes());
        payload[2..4].copy_from_slice(&vario_cms.to_be_bytes());

        Self::encode_frame(CRSF_FRAMETYPE_BARO_ALTITUDE, &payload, out)
    }
}
