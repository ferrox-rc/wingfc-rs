//! Binary Blackbox Flight Log Format.
//!
//! Compact, 32-byte packed frame format optimized for 50 Hz streaming to SPI / QSPI NOR flash.
//! Fits exactly 8 frames per 256-byte flash page without fragmentation or cross-page splits.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlackboxError {
    InvalidMagic,
    UnsupportedVersion,
    CrcMismatch,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C, packed)]
pub struct BlackboxFrame {
    pub magic: u8,
    pub version: u8,
    pub status_flags: u8,
    pub seq_lo: u8,
    pub timestamp_ms: u32,
    pub gyro_xyz_mrad: [i16; 3],
    pub accel_xyz_cms2: [i16; 3],
    pub pid_pitch: [i8; 3], // P, I, D terms scaled by 500 (-128..127 => -0.254..+0.254 mix)
    pub pid_roll: [i8; 3],  // P, I, D terms scaled by 500 (-128..127 => -0.254..+0.254 mix)
    pub packed_actuators: [u8; 4],
    pub aux: u8,
    pub crc8: u8,
}

impl BlackboxFrame {
    pub const SIZE: usize = 32;
    pub const MAGIC: u8 = 0xBB;
    pub const VERSION: u8 = 2;

    /// Packs 3 10-bit actuator pulse widths (1000..2023 us) into 4 bytes with 1 us resolution.
    #[inline]
    pub fn pack_actuators(servo1_us: u16, servo2_us: u16, throttle_us: u16) -> [u8; 4] {
        let s1 = (servo1_us.clamp(1000, 2023) - 1000) as u32;
        let s2 = (servo2_us.clamp(1000, 2023) - 1000) as u32;
        let thr = (throttle_us.clamp(1000, 2023) - 1000) as u32;
        let val = s1 | (s2 << 10) | (thr << 20);
        val.to_le_bytes()
    }

    /// Unpacks 4 bytes into (servo1_us, servo2_us, throttle_us).
    #[inline]
    pub fn unpack_actuators(packed: [u8; 4]) -> (u16, u16, u16) {
        let val = u32::from_le_bytes(packed);
        let s1 = ((val & 0x3FF) as u16) + 1000;
        let s2 = (((val >> 10) & 0x3FF) as u16) + 1000;
        let thr = (((val >> 20) & 0x3FF) as u16) + 1000;
        (s1, s2, thr)
    }

    /// Scales float PID terms [-0.254, 0.254] into i8 [-128, 127] with factor 500.0.
    #[inline]
    pub fn scale_pid_term(term: f32) -> i8 {
        let scaled = term * 500.0;
        if scaled > 127.0 {
            127
        } else if scaled < -128.0 {
            -128
        } else {
            scaled as i8
        }
    }

    /// Constructs a new BlackboxFrame, automatically computing the CRC8 checksum over bytes 0..31.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        status_flags: u8,
        seq_lo: u8,
        timestamp_ms: u32,
        gyro_xyz_mrad: [i16; 3],
        accel_xyz_cms2: [i16; 3],
        pid_pitch: [i8; 3],
        pid_roll: [i8; 3],
        servo1_us: u16,
        servo2_us: u16,
        throttle_us: u16,
        aux: u8,
    ) -> Self {
        let packed_actuators = Self::pack_actuators(servo1_us, servo2_us, throttle_us);
        let mut frame = Self {
            magic: Self::MAGIC,
            version: Self::VERSION,
            status_flags,
            seq_lo,
            timestamp_ms,
            gyro_xyz_mrad,
            accel_xyz_cms2,
            pid_pitch,
            pid_roll,
            packed_actuators,
            aux,
            crc8: 0,
        };
        frame.crc8 = frame.compute_crc();
        frame
    }

    /// Computes CRC8 over the first 31 bytes of the frame using polynomial 0x07 (CRC-8-ATM / ITU).
    pub fn compute_crc(&self) -> u8 {
        let bytes = self.to_bytes();
        crc8(&bytes[0..31])
    }

    /// Returns the actuators as (servo1_us, servo2_us, throttle_us).
    #[inline]
    pub fn actuators(&self) -> (u16, u16, u16) {
        Self::unpack_actuators(self.packed_actuators)
    }

    /// Serializes the frame into 32 raw bytes.
    pub fn to_bytes(&self) -> [u8; 32] {
        let mut buf = [0u8; 32];
        buf[0] = self.magic;
        buf[1] = self.version;
        buf[2] = self.status_flags;
        buf[3] = self.seq_lo;
        buf[4..8].copy_from_slice(&self.timestamp_ms.to_le_bytes());
        buf[8..10].copy_from_slice(&self.gyro_xyz_mrad[0].to_le_bytes());
        buf[10..12].copy_from_slice(&self.gyro_xyz_mrad[1].to_le_bytes());
        buf[12..14].copy_from_slice(&self.gyro_xyz_mrad[2].to_le_bytes());
        buf[14..16].copy_from_slice(&self.accel_xyz_cms2[0].to_le_bytes());
        buf[16..18].copy_from_slice(&self.accel_xyz_cms2[1].to_le_bytes());
        buf[18..20].copy_from_slice(&self.accel_xyz_cms2[2].to_le_bytes());
        buf[20] = self.pid_pitch[0] as u8;
        buf[21] = self.pid_pitch[1] as u8;
        buf[22] = self.pid_pitch[2] as u8;
        buf[23] = self.pid_roll[0] as u8;
        buf[24] = self.pid_roll[1] as u8;
        buf[25] = self.pid_roll[2] as u8;
        buf[26..30].copy_from_slice(&self.packed_actuators);
        buf[30] = self.aux;
        buf[31] = self.crc8;
        buf
    }

    /// Deserializes a 32-byte slice into a BlackboxFrame, validating magic, version, and CRC8.
    pub fn from_bytes(bytes: &[u8; 32]) -> Result<Self, BlackboxError> {
        if bytes[0] != Self::MAGIC {
            return Err(BlackboxError::InvalidMagic);
        }
        if bytes[1] != Self::VERSION {
            return Err(BlackboxError::UnsupportedVersion);
        }
        let expected_crc = crc8(&bytes[0..31]);
        if bytes[31] != expected_crc {
            return Err(BlackboxError::CrcMismatch);
        }

        let timestamp_ms = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let gx = i16::from_le_bytes([bytes[8], bytes[9]]);
        let gy = i16::from_le_bytes([bytes[10], bytes[11]]);
        let gz = i16::from_le_bytes([bytes[12], bytes[13]]);

        let ax = i16::from_le_bytes([bytes[14], bytes[15]]);
        let ay = i16::from_le_bytes([bytes[16], bytes[17]]);
        let az = i16::from_le_bytes([bytes[18], bytes[19]]);

        let pid_pitch = [bytes[20] as i8, bytes[21] as i8, bytes[22] as i8];
        let pid_roll = [bytes[23] as i8, bytes[24] as i8, bytes[25] as i8];

        let packed_actuators = [bytes[26], bytes[27], bytes[28], bytes[29]];

        Ok(Self {
            magic: bytes[0],
            version: bytes[1],
            status_flags: bytes[2],
            seq_lo: bytes[3],
            timestamp_ms,
            gyro_xyz_mrad: [gx, gy, gz],
            accel_xyz_cms2: [ax, ay, az],
            pid_pitch,
            pid_roll,
            packed_actuators,
            aux: bytes[30],
            crc8: bytes[31],
        })
    }
}

/// Standard CRC-8 calculation with polynomial 0x07 (CRC-8-ATM / ITU)
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0x00u8;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_actuator_packing_roundtrip() {
        for s1 in [1000, 1250, 1500, 1750, 2000] {
            for s2 in [1000, 1300, 1500, 1800, 2000] {
                for thr in [1000, 1100, 1500, 1900, 2000] {
                    let packed = BlackboxFrame::pack_actuators(s1, s2, thr);
                    let (u_s1, u_s2, u_thr) = BlackboxFrame::unpack_actuators(packed);
                    assert_eq!(s1, u_s1);
                    assert_eq!(s2, u_s2);
                    assert_eq!(thr, u_thr);
                }
            }
        }
    }

    #[test]
    fn test_blackbox_frame_size_and_roundtrip() {
        assert_eq!(core::mem::size_of::<BlackboxFrame>(), 32);

        let frame = BlackboxFrame::new(
            0x03,
            42,
            123456,
            [100, -200, 300],
            [10, 20, 981],
            [25, 5, -2],
            [30, 8, -4],
            1450,
            1550,
            1600,
            0x55,
        );

        let bytes = frame.to_bytes();
        assert_eq!(bytes.len(), 32);
        assert_eq!(bytes[0], BlackboxFrame::MAGIC);
        assert_eq!(bytes[1], BlackboxFrame::VERSION);

        let decoded = BlackboxFrame::from_bytes(&bytes).expect("Frame should decode cleanly");
        assert_eq!(frame, decoded);

        let (s1, s2, thr) = decoded.actuators();
        assert_eq!(s1, 1450);
        assert_eq!(s2, 1550);
        assert_eq!(thr, 1600);
        assert_eq!(decoded.pid_pitch, [25, 5, -2]);
        assert_eq!(decoded.pid_roll, [30, 8, -4]);
    }

    #[test]
    fn test_blackbox_crc_rejection() {
        let frame = BlackboxFrame::new(
            0x01,
            1,
            1000,
            [0, 0, 0],
            [0, 0, 981],
            [0, 0, 0],
            [0, 0, 0],
            1500,
            1500,
            1000,
            0,
        );
        let mut bytes = frame.to_bytes();

        // Corrupt a payload byte
        bytes[10] ^= 0xFF;
        assert_eq!(
            BlackboxFrame::from_bytes(&bytes),
            Err(BlackboxError::CrcMismatch)
        );

        // Corrupt CRC itself
        bytes[10] ^= 0xFF; // restore
        bytes[31] ^= 0x01;
        assert_eq!(
            BlackboxFrame::from_bytes(&bytes),
            Err(BlackboxError::CrcMismatch)
        );
    }
}
