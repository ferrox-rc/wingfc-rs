//! High-performance, zero-allocation GPS protocol parser and navigation algorithms.
//!
//! Designed for `#![no_std]` flight control applications. Supports standard NMEA 0183
//! sentences (GGA, RMC) with hardware-accelerated coordinate projection, home-point
//! latching, and Return-to-Home (RTH) waypoint calculations.

use crate::math::DEG_TO_RAD;

/// GPS fix quality classification
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GpsFixType {
    NoFix = 0,
    Fix2D = 1,
    Fix3D = 2,
    FixDgps = 3,
}

/// Instantaneous decoded GPS telemetry packet
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GpsData {
    /// Latitude in decimal degrees (-90.0 to +90.0, North positive)
    pub latitude_deg: f32,
    /// Longitude in decimal degrees (-180.0 to +180.0, East positive)
    pub longitude_deg: f32,
    /// Altitude above Mean Sea Level (meters)
    pub altitude_msl_m: f32,
    /// Speed Over Ground (meters per second)
    pub ground_speed_m_s: f32,
    /// Course Over Ground / ground track (degrees, 0.0 to 359.9)
    pub course_deg: f32,
    /// Number of actively tracked satellites
    pub satellites: u8,
    /// Horizontal Dilution of Precision (HDOP)
    pub hdop: f32,
    /// Current fix classification
    pub fix_type: GpsFixType,
}

impl Default for GpsData {
    fn default() -> Self {
        Self {
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_msl_m: 0.0,
            ground_speed_m_s: 0.0,
            course_deg: 0.0,
            satellites: 0,
            hdop: 99.9,
            fix_type: GpsFixType::NoFix,
        }
    }
}

/// Return-to-Home (RTH) navigation state and guidance command
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RthGuidance {
    /// True bearing to home point (degrees, 0 to 360)
    pub bearing_to_home_deg: f32,
    /// Horizontal line-of-sight distance to home point (meters)
    pub distance_to_home_m: f32,
    /// Commanded heading error relative to current track (-180 to +180 degrees)
    pub heading_error_deg: f32,
    /// Commanded target altitude AGL for safe return (meters)
    pub target_altitude_m: f32,
}

/// Autonomous GPS navigation manager managing home point and RTH guidance
pub struct GpsNavigation {
    home_lat: f32,
    home_lon: f32,
    home_alt: f32,
    home_latched: bool,
    min_satellites: u8,
    max_hdop: f32,
    safe_rth_altitude_m: f32,
}

impl GpsNavigation {
    /// Creates a new GPS navigation instance.
    pub fn new(min_satellites: u8, max_hdop: f32, safe_rth_altitude_m: f32) -> Self {
        Self {
            home_lat: 0.0,
            home_lon: 0.0,
            home_alt: 0.0,
            home_latched: false,
            min_satellites,
            max_hdop,
            safe_rth_altitude_m,
        }
    }

    /// True if the home position has been acquired and latched.
    pub fn is_home_latched(&self) -> bool {
        self.home_latched
    }

    /// Latched home coordinates (latitude, longitude, altitude MSL).
    pub fn home_coordinates(&self) -> Option<(f32, f32, f32)> {
        if self.home_latched {
            Some((self.home_lat, self.home_lon, self.home_alt))
        } else {
            None
        }
    }

    /// Attempts to latch the home coordinate if conditions meet satellite and HDOP criteria.
    pub fn try_latch_home(&mut self, gps: &GpsData) -> bool {
        if self.home_latched {
            return true;
        }

        if (gps.fix_type == GpsFixType::Fix3D || gps.fix_type == GpsFixType::FixDgps)
            && gps.satellites >= self.min_satellites
            && gps.hdop <= self.max_hdop
            && gps.hdop > 0.0
        {
            self.home_lat = gps.latitude_deg;
            self.home_lon = gps.longitude_deg;
            self.home_alt = gps.altitude_msl_m;
            self.home_latched = true;
            return true;
        }

        false
    }

    /// Computes great-circle horizontal distance (meters) and initial true bearing (degrees)
    /// between two coordinates using equirectangular flat-Earth projection.
    ///
    /// Accurate to within < 0.1% for distances under 100 km.
    pub fn distance_and_bearing(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> (f32, f32) {
        let earth_radius_m: f32 = 6_371_000.0;
        let lat_mid = (lat1 + lat2) * 0.5 * DEG_TO_RAD;

        let d_lat = (lat2 - lat1) * DEG_TO_RAD;
        let d_lon = (lon2 - lon1) * DEG_TO_RAD;

        let cos_lat = libm::cosf(lat_mid);
        let x = d_lon * cos_lat;
        let y = d_lat;

        let dist_m = libm::sqrtf(x * x + y * y) * earth_radius_m;

        // Bearing from (lat1, lon1) to (lat2, lon2)
        let bearing_rad = libm::atan2f(x, y);
        let mut bearing_deg = bearing_rad * (180.0 / core::f32::consts::PI);
        if bearing_deg < 0.0 {
            bearing_deg += 360.0;
        }

        (dist_m, bearing_deg)
    }

    /// Calculates Return-to-Home guidance vector and heading error from current GPS state.
    pub fn calculate_rth_guidance(&self, current: &GpsData) -> Option<RthGuidance> {
        if !self.home_latched {
            return None;
        }

        let (dist, bearing) = Self::distance_and_bearing(
            current.latitude_deg,
            current.longitude_deg,
            self.home_lat,
            self.home_lon,
        );

        // Heading error: shortest angular path between current course and target bearing
        let mut err_deg = bearing - current.course_deg;
        while err_deg > 180.0 {
            err_deg -= 360.0;
        }
        while err_deg < -180.0 {
            err_deg += 360.0;
        }

        // Return altitude target: home altitude + safe RTH climb margin
        let target_alt = self.home_alt + self.safe_rth_altitude_m;

        Some(RthGuidance {
            bearing_to_home_deg: bearing,
            distance_to_home_m: dist,
            heading_error_deg: err_deg,
            target_altitude_m: target_alt,
        })
    }
}

/// Zero-allocation streaming NMEA 0183 parser for GPS receivers.
pub struct NmeaParser {
    buf: [u8; 96],
    buf_len: usize,
    in_sentence: bool,
    latest_data: GpsData,
}

impl Default for NmeaParser {
    fn default() -> Self {
        Self::new()
    }
}

impl NmeaParser {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; 96],
            buf_len: 0,
            in_sentence: false,
            latest_data: GpsData {
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_msl_m: 0.0,
                ground_speed_m_s: 0.0,
                course_deg: 0.0,
                satellites: 0,
                hdop: 99.9,
                fix_type: GpsFixType::NoFix,
            },
        }
    }

    /// Read-only reference to the latest validated GPS state.
    pub fn data(&self) -> &GpsData {
        &self.latest_data
    }

    /// Streams incoming serial bytes into the parser.
    ///
    /// Returns `Some(&GpsData)` when a complete, valid NMEA sentence has updated the fix.
    pub fn feed_byte(&mut self, byte: u8) -> Option<&GpsData> {
        if byte == b'$' {
            self.buf_len = 0;
            self.in_sentence = true;
            return None;
        }

        if !self.in_sentence {
            return None;
        }

        if byte == b'\r' || byte == b'\n' {
            if self.buf_len > 0 {
                let updated = self.process_sentence();
                self.in_sentence = false;
                self.buf_len = 0;
                if updated {
                    return Some(&self.latest_data);
                }
            }
            self.in_sentence = false;
            self.buf_len = 0;
            return None;
        }

        if self.buf_len < self.buf.len() {
            self.buf[self.buf_len] = byte;
            self.buf_len += 1;
        } else {
            // Buffer overflow: sentence was malformed or too long
            self.in_sentence = false;
            self.buf_len = 0;
        }

        None
    }

    /// Validates checksum and dispatches supported NMEA sentences (GGA, RMC).
    fn process_sentence(&mut self) -> bool {
        let sentence = &self.buf[..self.buf_len];

        // Find '*' checksum marker
        let star_pos = match sentence.iter().position(|&b| b == b'*') {
            Some(pos) => pos,
            None => return false,
        };

        if star_pos + 2 >= sentence.len() {
            return false;
        }

        // Verify XOR checksum
        let mut expected_cs: u8 = 0;
        for &b in &sentence[..star_pos] {
            expected_cs ^= b;
        }

        let actual_cs = match parse_hex_byte(&sentence[star_pos + 1..star_pos + 3]) {
            Some(cs) => cs,
            None => return false,
        };

        if expected_cs != actual_cs {
            return false;
        }

        // Sentence payload before checksum
        let payload = &sentence[..star_pos];

        // Dispatches sentence based on talker & sentence ID (e.g. GNGGA, GPGGA, GNRMC, GPRMC)
        if payload.len() >= 5 {
            let sentence_id = &payload[2..5];
            if sentence_id == b"GGA" {
                return Self::parse_gga(payload, &mut self.latest_data);
            } else if sentence_id == b"RMC" {
                return Self::parse_rmc(payload, &mut self.latest_data);
            }
        }

        false
    }

    /// Parses `$..GGA` sentence (Global Positioning System Fix Data).
    ///
    /// Example: `$GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47`
    fn parse_gga(payload: &[u8], latest_data: &mut GpsData) -> bool {
        let mut fields = [b"".as_slice(); 15];
        let field_count = split_fields(payload, &mut fields);
        if field_count < 10 {
            return false;
        }

        // Field 2: Latitude (ddmm.mmmm)
        // Field 3: N/S
        // Field 4: Longitude (dddmm.mmmm)
        // Field 5: E/W
        let lat = parse_coordinate(fields[2], fields[3], true);
        let lon = parse_coordinate(fields[4], fields[5], false);

        // Field 6: Fix Quality (0 = Invalid, 1 = GPS Fix, 2 = DGPS Fix)
        let fix_byte = fields[6].first().copied().unwrap_or(b'0');
        let fix_type = match fix_byte {
            b'1' => GpsFixType::Fix3D,
            b'2' => GpsFixType::FixDgps,
            _ => GpsFixType::NoFix,
        };

        // Field 7: Satellite Count
        let sats = parse_int(fields[7]).unwrap_or(0) as u8;

        // Field 8: HDOP
        let hdop = parse_float(fields[8]).unwrap_or(99.9);

        // Field 9: Altitude MSL
        let alt = parse_float(fields[9]).unwrap_or(0.0);

        if let (Some(lat_deg), Some(lon_deg)) = (lat, lon) {
            latest_data.latitude_deg = lat_deg;
            latest_data.longitude_deg = lon_deg;
            latest_data.altitude_msl_m = alt;
            latest_data.satellites = sats;
            latest_data.hdop = hdop;
            latest_data.fix_type = fix_type;
            return true;
        }

        false
    }

    /// Parses `$..RMC` sentence (Recommended Minimum Specific GPS Data).
    ///
    /// Example: `$GNRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A`
    fn parse_rmc(payload: &[u8], latest_data: &mut GpsData) -> bool {
        let mut fields = [b"".as_slice(); 13];
        let field_count = split_fields(payload, &mut fields);
        if field_count < 9 {
            return false;
        }

        // Field 2: Status ('A' = valid, 'V' = void/warning)
        let status = fields[2].first().copied().unwrap_or(b'V');
        if status != b'A' {
            latest_data.fix_type = GpsFixType::NoFix;
            return false;
        }

        // Field 3..6: Latitude / Longitude
        let lat = parse_coordinate(fields[3], fields[4], true);
        let lon = parse_coordinate(fields[5], fields[6], false);

        // Field 7: Speed Over Ground (knots -> convert to m/s: 1 knot = 0.514444 m/s)
        let speed_knots = parse_float(fields[7]).unwrap_or(0.0);
        let speed_m_s = speed_knots * 0.514444;

        // Field 8: Track Angle / Course Over Ground (degrees)
        let course = parse_float(fields[8]).unwrap_or(0.0);

        if let (Some(lat_deg), Some(lon_deg)) = (lat, lon) {
            latest_data.latitude_deg = lat_deg;
            latest_data.longitude_deg = lon_deg;
            latest_data.ground_speed_m_s = speed_m_s;
            latest_data.course_deg = course;
            if latest_data.fix_type == GpsFixType::NoFix {
                latest_data.fix_type = GpsFixType::Fix2D;
            }
            return true;
        }

        false
    }
}

// =============================================================================
// Internal Zero-Allocation Parsing Utilities
// =============================================================================

/// Splits a comma-separated byte slice into pre-allocated field buffers.
fn split_fields<'a>(payload: &'a [u8], fields: &mut [&'a [u8]]) -> usize {
    let mut count = 0;
    let mut start = 0;

    for (i, &b) in payload.iter().enumerate() {
        if b == b',' {
            if count < fields.len() {
                fields[count] = &payload[start..i];
                count += 1;
            }
            start = i + 1;
        }
    }

    if count < fields.len() && start <= payload.len() {
        fields[count] = &payload[start..];
        count += 1;
    }

    count
}

/// Converts NMEA ddmm.mmmm (lat) or dddmm.mmmm (lon) to signed decimal degrees.
fn parse_coordinate(coord_bytes: &[u8], hemi_bytes: &[u8], is_lat: bool) -> Option<f32> {
    if coord_bytes.is_empty() || hemi_bytes.is_empty() {
        return None;
    }

    let deg_digits = if is_lat { 2 } else { 3 };
    if coord_bytes.len() < deg_digits {
        return None;
    }

    let degrees = parse_int(&coord_bytes[..deg_digits])? as f32;
    let minutes = parse_float(&coord_bytes[deg_digits..])?;

    let mut decimal = degrees + (minutes / 60.0);

    let hemi = hemi_bytes.first().copied()?;
    if hemi == b'S' || hemi == b'W' {
        decimal = -decimal;
    }

    Some(decimal)
}

/// Parses a float from an ASCII slice without heap allocation or standard library.
fn parse_float(bytes: &[u8]) -> Option<f32> {
    if bytes.is_empty() {
        return None;
    }

    let mut is_negative = false;
    let mut idx = 0;

    if bytes[0] == b'-' {
        is_negative = true;
        idx += 1;
    } else if bytes[0] == b'+' {
        idx += 1;
    }

    let mut int_part: u32 = 0;
    let mut frac_part: u32 = 0;
    let mut frac_div: f32 = 1.0;
    let mut in_frac = false;

    while idx < bytes.len() {
        let b = bytes[idx];
        if b == b'.' {
            in_frac = true;
        } else if b.is_ascii_digit() {
            let digit = (b - b'0') as u32;
            if !in_frac {
                int_part = int_part.checked_mul(10)?.checked_add(digit)?;
            } else {
                frac_part = frac_part.checked_mul(10)?.checked_add(digit)?;
                frac_div *= 10.0;
            }
        } else {
            break;
        }
        idx += 1;
    }

    let val = (int_part as f32) + ((frac_part as f32) / frac_div);
    Some(if is_negative { -val } else { val })
}

/// Parses an unsigned integer from an ASCII slice.
fn parse_int(bytes: &[u8]) -> Option<u32> {
    if bytes.is_empty() {
        return None;
    }

    let mut val: u32 = 0;
    for &b in bytes {
        if b.is_ascii_digit() {
            val = val.checked_mul(10)?.checked_add((b - b'0') as u32)?;
        } else {
            return None;
        }
    }
    Some(val)
}

/// Parses two hexadecimal characters into a single byte.
fn parse_hex_byte(bytes: &[u8]) -> Option<u8> {
    if bytes.len() != 2 {
        return None;
    }

    let hi = hex_digit(bytes[0])?;
    let lo = hex_digit(bytes[1])?;

    Some((hi << 4) | lo)
}

fn hex_digit(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}
