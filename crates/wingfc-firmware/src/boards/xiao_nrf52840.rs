use wingfc_core::math::BoardOrientation;

/// Total capacity of onboard Puya P25Q16H QSPI NOR flash: 2 MB (2,097,152 bytes).
pub const NRF52840_FLASH_CAPACITY_BYTES: u32 = 2 * 1024 * 1024;

/// Board identification string
pub const BOARD_NAME: &'static str = "Seeed Studio XIAO nRF52840 Sense";

/// Default IMU mounting orientation for the XIAO nRF52840 board
pub const DEFAULT_ORIENTATION: BoardOrientation = BoardOrientation::Default;

/// Target microcontroller architecture name
pub const TARGET_TRIPLE: &'static str = "thumbv7em-none-eabihf";
