//! On-chip hardware peripheral traits for WingFC flight controllers.
//!
//! Defines MCU-agnostic contracts for PWM actuator outputs (servos and ESCs),
//! external NOR flash storage (QSPI / XSPI Blackbox), and visual status indicators.
//! Concrete implementations live inside `wingfc-firmware::boards` (e.g. `nrf52840` or `stm32c5`).

use core::ops::{Deref, DerefMut};

/// Multi-channel Servo PWM actuator interface (1000..2000 us standard pulses).
pub trait ServoOutput {
    /// Updates the pulse width in microseconds for a servo channel (0..3).
    fn set_channel_us(&mut self, channel: usize, pulse_us: u16);
}

/// Motor ESC throttle output interface (1000..2000 us PWM or digital protocol).
pub trait MotorOutput {
    /// Updates the throttle command in microseconds (1000..2000 us) for a motor channel (0..1).
    fn set_throttle_us(&mut self, channel: usize, pulse_us: u16);
}

/// Standard page size in bytes for NOR flash program operations (256 bytes = 8 Blackbox frames).
pub const FLASH_PAGE_SIZE: usize = 256;

/// Standard sector size in bytes for NOR flash erase operations (4 KB = 16 pages).
pub const FLASH_SECTOR_SIZE: usize = 4096;

/// 4-byte aligned 256-byte page buffer required by hardware DMA controllers (EasyDMA / GPDMA).
#[derive(Clone, Copy)]
#[repr(align(4))]
pub struct PageBuffer(pub [u8; FLASH_PAGE_SIZE]);

impl PageBuffer {
    pub const fn new() -> Self {
        Self([0u8; FLASH_PAGE_SIZE])
    }
}

impl Default for PageBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for PageBuffer {
    type Target = [u8; FLASH_PAGE_SIZE];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for PageBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum FlashError {
    DeviceNotFound,
    InvalidJedecId([u8; 3]),
    OutOfBounds,
    WriteError,
    ReadError,
    EraseError,
}

/// Asynchronous NOR Flash storage interface for high-speed Blackbox flight logging.
///
/// Implemented by on-chip QSPI (`P25Q16H` 2 MB on XIAO nRF52840) and XSPI/OCTOSPI
/// (`PY25Q128HA` 16 MB on XIAO STM32C5) peripheral drivers.
#[allow(async_fn_in_trait)]
pub trait NorFlashStorage {
    /// Total flash capacity in bytes (e.g., 2 MB on XIAO Sense, 16 MB on XIAO STM32C5).
    const CAPACITY_BYTES: u32;

    /// Reads the 3-byte JEDEC manufacturer and device ID.
    async fn read_jedec_id(&mut self) -> Result<[u8; 3], FlashError>;

    /// Reads a 256-byte page via DMA into a 4-byte aligned buffer.
    async fn read_page(&mut self, address: u32, buf: &mut PageBuffer) -> Result<(), FlashError>;

    /// Writes a 256-byte page via DMA from a 4-byte aligned buffer.
    async fn write_page(&mut self, address: u32, buf: &PageBuffer) -> Result<(), FlashError>;

    /// Erases a 4 KB sector (address must be 4096-byte aligned).
    async fn erase_sector(&mut self, address: u32) -> Result<(), FlashError>;

    /// Reads an arbitrary 4-byte aligned slice from flash.
    async fn read_aligned(&mut self, address: u32, buf: &mut [u8]) -> Result<(), FlashError>;
}
