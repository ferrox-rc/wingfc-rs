//! External Barometer drivers (Infineon DPS310 & Bosch BMP280) on TWISPI1.
//!
//! Provides support for explicit external I2C digital barometers on pins D4 (P0.04 SDA)
//! and D5 (P0.05 SCL).

use embassy_nrf::twim::Twim;
use embassy_time::{with_timeout, Duration, Timer};
use libm::powf;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BaroModel {
    Dps310,
    Bmp280,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum BaroError {
    I2cError,
}

/// Convert pressure in Pascals (Pa) and reference sea-level / ground pressure (Pa)
/// to relative altitude in meters using standard hypsometric formula.
#[inline]
pub fn pressure_to_altitude(pressure_pa: f32, ground_pressure_pa: f32) -> f32 {
    if pressure_pa <= 0.0 || ground_pressure_pa <= 0.0 {
        return 0.0;
    }
    // h = 44330.0 * (1.0 - (P / P0) ^ 0.190295)
    44330.0 * (1.0 - powf(pressure_pa / ground_pressure_pa, 0.190295))
}

// =============================================================================
// Infineon DPS310 Driver Implementation
// =============================================================================

// DPS310 Registers
const DPS310_REG_PRS_B2: u8 = 0x00;
const DPS310_REG_PRS_CFG: u8 = 0x06;
const DPS310_REG_TMP_CFG: u8 = 0x07;
const DPS310_REG_MEAS_CFG: u8 = 0x08;
const DPS310_REG_CFG_REG: u8 = 0x09;
const DPS310_REG_RESET: u8 = 0x0C;
const DPS310_REG_PROD_ID: u8 = 0x0D;
const DPS310_REG_COEF: u8 = 0x10;

#[derive(Debug, Default, Clone, Copy)]
struct Dps310Coeffs {
    c0: i32,
    c1: i32,
    c00: i32,
    c10: i32,
    c01: i32,
    c11: i32,
    c20: i32,
    c21: i32,
    c30: i32,
}

pub struct Dps310Driver {
    addr: u8,
    coeffs: Dps310Coeffs,
    k_p: f32,
    k_t: f32,
}

impl Dps310Driver {
    pub fn new(addr: u8) -> Self {
        Self {
            addr,
            coeffs: Dps310Coeffs::default(),
            k_p: 1.0,
            k_t: 1.0,
        }
    }

    pub async fn init(&mut self, twim: &mut Twim<'_>) -> Result<(), BaroError> {
        Timer::after(Duration::from_millis(40)).await;

        // 1. Soft Reset
        let reset_cmd = [DPS310_REG_RESET, 0x89];
        let _ = with_timeout(Duration::from_millis(50), twim.write(self.addr, &reset_cmd)).await;

        Timer::after(Duration::from_millis(40)).await;

        // 2. Read Product ID (Register 0x0D: Lower nibble should be 0x0 or 0x1)
        let mut prod_id = [0u8; 1];
        with_timeout(
            Duration::from_millis(50),
            twim.write_read(self.addr, &[DPS310_REG_PROD_ID], &mut prod_id),
        )
        .await
        .map_err(|_| BaroError::I2cError)?
        .map_err(|_| BaroError::I2cError)?;

        if (prod_id[0] & 0x0F) != 0x00 && (prod_id[0] & 0x0F) != 0x01 && (prod_id[0] >> 4) != 0x01 {
            defmt::warn!("DPS310 Prod ID check: 0x{:02X}", prod_id[0]);
        }

        // 3. Wait for sensor ready and coeff ready (MEAS_CFG: bits 4-7)
        let mut ready = false;
        for _ in 0..20 {
            let mut status = [0u8; 1];
            if let Ok(Ok(())) = with_timeout(
                Duration::from_millis(20),
                twim.write_read(self.addr, &[DPS310_REG_MEAS_CFG], &mut status),
            )
            .await
            {
                // Bit 6: SENSOR_RDY, Bit 7: COEF_RDY
                if (status[0] & 0xC0) == 0xC0 {
                    ready = true;
                    break;
                }
            }
            Timer::after(Duration::from_millis(10)).await;
        }

        if !ready {
            defmt::warn!("DPS310: Sensor/Coeff ready wait timed out, attempting coeff read");
        }

        // 4. Read 18 bytes of calibration coefficients (0x10..0x21)
        let mut c_buf = [0u8; 18];
        with_timeout(
            Duration::from_millis(50),
            twim.write_read(self.addr, &[DPS310_REG_COEF], &mut c_buf),
        )
        .await
        .map_err(|_| BaroError::I2cError)?
        .map_err(|_| BaroError::I2cError)?;

        self.parse_coeffs(&c_buf);

        // 5. Configure Pressure & Temperature sampling rates
        // PM_RATE: 64 Hz (0x06 << 4), PM_PRC: 16x oversampling (0x04) -> 0x64
        // Scale factor for 16x oversampling: 253952
        let prs_cfg = [DPS310_REG_PRS_CFG, 0x64];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &prs_cfg))
            .await
            .map_err(|_| BaroError::I2cError)?
            .map_err(|_| BaroError::I2cError)?;
        self.k_p = 253952.0;

        // TMP_RATE: 32 Hz (0x05 << 4), TMP_PRC: 8x oversampling (0x03), TMP_EXT: 0x80 (internal ASIC) -> 0x53
        // Scale factor for 8x oversampling: 524288
        let tmp_cfg = [DPS310_REG_TMP_CFG, 0x53];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &tmp_cfg))
            .await
            .map_err(|_| BaroError::I2cError)?
            .map_err(|_| BaroError::I2cError)?;
        self.k_t = 524288.0;

        // CFG_REG: Temperature & Pressure bit-shift enable when oversampling > 8x (PRS_SHIFT: bit 2, TMP_SHIFT: bit 3)
        let cfg_shift = [DPS310_REG_CFG_REG, 0x0C];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &cfg_shift))
            .await
            .map_err(|_| BaroError::I2cError)?
            .map_err(|_| BaroError::I2cError)?;

        // 6. Start continuous background measurement (MEAS_CFG: Mode 7 = Continuous Temp & Pressure)
        let mode_cmd = [DPS310_REG_MEAS_CFG, 0x07];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &mode_cmd))
            .await
            .map_err(|_| BaroError::I2cError)?
            .map_err(|_| BaroError::I2cError)?;

        defmt::info!(
            "DPS310 Barometer initialized on external I2C (addr 0x{:02X})",
            self.addr
        );
        Ok(())
    }

    fn parse_coeffs(&mut self, buf: &[u8; 18]) {
        // c0: 12-bit signed [3..0 of b0, b1]
        let mut c0 = ((buf[0] as i32) << 4) | (((buf[1] as i32) >> 4) & 0x0F);
        if c0 & 0x800 != 0 {
            c0 -= 1 << 12;
        }

        // c1: 12-bit signed [b1 lower 4, b2]
        let mut c1 = (((buf[1] as i32) & 0x0F) << 8) | (buf[2] as i32);
        if c1 & 0x800 != 0 {
            c1 -= 1 << 12;
        }

        // c00: 20-bit signed [b3, b4, b5 upper 4]
        let mut c00 =
            ((buf[3] as i32) << 12) | ((buf[4] as i32) << 4) | (((buf[5] as i32) >> 4) & 0x0F);
        if c00 & 0x80000 != 0 {
            c00 -= 1 << 20;
        }

        // c10: 20-bit signed [b5 lower 4, b6, b7]
        let mut c10 = (((buf[5] as i32) & 0x0F) << 16) | ((buf[6] as i32) << 8) | (buf[7] as i32);
        if c10 & 0x80000 != 0 {
            c10 -= 1 << 20;
        }

        // c01: 16-bit signed [b8, b9]
        let c01 = ((buf[8] as i16) << 8 | (buf[9] as i16)) as i32;

        // c11: 16-bit signed [b10, b11]
        let c11 = ((buf[10] as i16) << 8 | (buf[11] as i16)) as i32;

        // c20: 16-bit signed [b12, b13]
        let c20 = ((buf[12] as i16) << 8 | (buf[13] as i16)) as i32;

        // c21: 16-bit signed [b14, b15]
        let c21 = ((buf[14] as i16) << 8 | (buf[15] as i16)) as i32;

        // c30: 16-bit signed [b16, b17]
        let c30 = ((buf[16] as i16) << 8 | (buf[17] as i16)) as i32;

        self.coeffs = Dps310Coeffs {
            c0,
            c1,
            c00,
            c10,
            c01,
            c11,
            c20,
            c21,
            c30,
        };
    }

    /// Read raw pressure and temperature registers and compute compensated values.
    /// Returns: (Pressure in Pascals, Temperature in Celsius)
    pub async fn read(&mut self, twim: &mut Twim<'_>) -> Result<(f32, f32), BaroError> {
        // Read 6 bytes starting at PRS_B2 (0x00..0x05): PRS [B2, B1, B0], TMP [B2, B1, B0]
        let mut buf = [0u8; 6];
        with_timeout(
            Duration::from_millis(15),
            twim.write_read(self.addr, &[DPS310_REG_PRS_B2], &mut buf),
        )
        .await
        .map_err(|_| BaroError::I2cError)?
        .map_err(|_| BaroError::I2cError)?;

        // Raw pressure (24-bit 2's complement)
        let mut raw_p = ((buf[0] as i32) << 16) | ((buf[1] as i32) << 8) | (buf[2] as i32);
        if raw_p & 0x800000 != 0 {
            raw_p -= 1 << 24;
        }

        // Raw temperature (24-bit 2's complement)
        let mut raw_t = ((buf[3] as i32) << 16) | ((buf[4] as i32) << 8) | (buf[5] as i32);
        if raw_t & 0x800000 != 0 {
            raw_t -= 1 << 24;
        }

        let p_scaled = (raw_p as f32) / self.k_p;
        let t_scaled = (raw_t as f32) / self.k_t;

        // Temperature compensation:
        // T_comp = c0 * 0.5 + c1 * T_scaled
        let c = &self.coeffs;
        let temp_c = (c.c0 as f32) * 0.5 + (c.c1 as f32) * t_scaled;

        // Pressure compensation polynomial:
        // P_comp = c00 + P_scaled * (c10 + P_scaled * (c20 + P_scaled * c30))
        //        + T_scaled * c01 + T_scaled * P_scaled * (c11 + P_scaled * c21)
        let p_comp = (c.c00 as f32)
            + p_scaled * ((c.c10 as f32) + p_scaled * ((c.c20 as f32) + p_scaled * (c.c30 as f32)))
            + t_scaled * (c.c01 as f32)
            + t_scaled * p_scaled * ((c.c11 as f32) + p_scaled * (c.c21 as f32));

        Ok((p_comp, temp_c))
    }
}

// =============================================================================
// Bosch BMP280 Driver Implementation
// =============================================================================

// BMP280 Registers
const BMP280_REG_PRESS_MSB: u8 = 0xF7;
const BMP280_REG_CONFIG: u8 = 0xF5;
const BMP280_REG_CTRL_MEAS: u8 = 0xF4;
const BMP280_REG_RESET: u8 = 0xE0;
const BMP280_REG_ID: u8 = 0xD0;
const BMP280_REG_CALIB00: u8 = 0x88;

#[derive(Debug, Default, Clone, Copy)]
struct Bmp280Calib {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
}

pub struct Bmp280Driver {
    addr: u8,
    calib: Bmp280Calib,
    t_fine: i32,
}

impl Bmp280Driver {
    pub fn new(addr: u8) -> Self {
        Self {
            addr,
            calib: Bmp280Calib::default(),
            t_fine: 0,
        }
    }

    pub async fn init(&mut self, twim: &mut Twim<'_>) -> Result<(), BaroError> {
        Timer::after(Duration::from_millis(20)).await;

        // 1. Soft Reset
        let reset_cmd = [BMP280_REG_RESET, 0xB6];
        let _ = with_timeout(Duration::from_millis(50), twim.write(self.addr, &reset_cmd)).await;

        Timer::after(Duration::from_millis(30)).await;

        // 2. Read Chip ID (Register 0xD0: Expected 0x58 for BMP280)
        let mut chip_id = [0u8; 1];
        with_timeout(
            Duration::from_millis(50),
            twim.write_read(self.addr, &[BMP280_REG_ID], &mut chip_id),
        )
        .await
        .map_err(|_| BaroError::I2cError)?
        .map_err(|_| BaroError::I2cError)?;

        if chip_id[0] != 0x58 && chip_id[0] != 0x56 && chip_id[0] != 0x57 && chip_id[0] != 0x60 {
            defmt::warn!("BMP280 Chip ID check: 0x{:02X}", chip_id[0]);
        }

        // 3. Read 24 calibration bytes (0x88..0xA1)
        let mut cal = [0u8; 24];
        with_timeout(
            Duration::from_millis(50),
            twim.write_read(self.addr, &[BMP280_REG_CALIB00], &mut cal),
        )
        .await
        .map_err(|_| BaroError::I2cError)?
        .map_err(|_| BaroError::I2cError)?;

        self.calib = Bmp280Calib {
            dig_t1: u16::from_le_bytes([cal[0], cal[1]]),
            dig_t2: i16::from_le_bytes([cal[2], cal[3]]),
            dig_t3: i16::from_le_bytes([cal[4], cal[5]]),
            dig_p1: u16::from_le_bytes([cal[6], cal[7]]),
            dig_p2: i16::from_le_bytes([cal[8], cal[9]]),
            dig_p3: i16::from_le_bytes([cal[10], cal[11]]),
            dig_p4: i16::from_le_bytes([cal[12], cal[13]]),
            dig_p5: i16::from_le_bytes([cal[14], cal[15]]),
            dig_p6: i16::from_le_bytes([cal[16], cal[17]]),
            dig_p7: i16::from_le_bytes([cal[18], cal[19]]),
            dig_p8: i16::from_le_bytes([cal[20], cal[21]]),
            dig_p9: i16::from_le_bytes([cal[22], cal[23]]),
        };

        // 4. Configure Filter & Standby Time
        // t_sb = 62.5ms (0x01 << 5), filter = 16 (0x04 << 2) -> 0x30
        let cfg_cmd = [BMP280_REG_CONFIG, 0x30];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &cfg_cmd))
            .await
            .map_err(|_| BaroError::I2cError)?
            .map_err(|_| BaroError::I2cError)?;

        // 5. Configure CTRL_MEAS: osrs_t = 2x (0x02 << 5), osrs_p = 16x (0x05 << 2), mode = Normal (0x03) -> 0x57
        let ctrl_cmd = [BMP280_REG_CTRL_MEAS, 0x57];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &ctrl_cmd))
            .await
            .map_err(|_| BaroError::I2cError)?
            .map_err(|_| BaroError::I2cError)?;

        defmt::info!(
            "BMP280 Barometer initialized on external I2C (addr 0x{:02X})",
            self.addr
        );
        Ok(())
    }

    /// Read burst 6 bytes starting at 0xF7 (Press MSB..XLSB, Temp MSB..XLSB)
    /// Returns: (Pressure in Pascals, Temperature in Celsius)
    pub async fn read(&mut self, twim: &mut Twim<'_>) -> Result<(f32, f32), BaroError> {
        let mut buf = [0u8; 6];
        with_timeout(
            Duration::from_millis(15),
            twim.write_read(self.addr, &[BMP280_REG_PRESS_MSB], &mut buf),
        )
        .await
        .map_err(|_| BaroError::I2cError)?
        .map_err(|_| BaroError::I2cError)?;

        let adc_p = ((buf[0] as i32) << 12) | ((buf[1] as i32) << 4) | ((buf[2] as i32) >> 4);
        let adc_t = ((buf[3] as i32) << 12) | ((buf[4] as i32) << 4) | ((buf[5] as i32) >> 4);

        // Bosch temperature compensation
        let c = &self.calib;
        let var1_t = (((adc_t >> 3) - ((c.dig_t1 as i32) << 1)) * (c.dig_t2 as i32)) >> 11;
        let var2_t = (((((adc_t >> 4) - (c.dig_t1 as i32)) * ((adc_t >> 4) - (c.dig_t1 as i32)))
            >> 12)
            * (c.dig_t3 as i32))
            >> 14;
        self.t_fine = var1_t + var2_t;
        let temp_c = ((self.t_fine * 5 + 128) >> 8) as f32 / 100.0;

        // Bosch pressure compensation (64-bit integer formula)
        let mut var1_p = (self.t_fine as i64) - 128000;
        let mut var2_p = var1_p * var1_p * (c.dig_p6 as i64);
        var2_p += (var1_p * (c.dig_p5 as i64)) << 17;
        var2_p += (c.dig_p4 as i64) << 35;
        var1_p =
            ((var1_p * var1_p * (c.dig_p3 as i64)) >> 8) + ((var1_p * (c.dig_p2 as i64)) << 12);
        var1_p = (((1i64 << 47) + var1_p) * (c.dig_p1 as i64)) >> 33;

        if var1_p == 0 {
            return Ok((0.0, temp_c));
        }

        let mut p = 1048576 - (adc_p as i64);
        p = (((p << 31) - var2_p) * 3125) / var1_p;
        var1_p = ((c.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
        var2_p = ((c.dig_p8 as i64) * p) >> 19;
        p = ((p + var1_p + var2_p) >> 8) + ((c.dig_p7 as i64) << 4);

        let pressure_pa = (p as f32) / 256.0;
        Ok((pressure_pa, temp_c))
    }
}
