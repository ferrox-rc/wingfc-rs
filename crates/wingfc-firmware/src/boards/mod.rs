//! Board hardware abstraction layer modules for supported flight controllers.

pub mod xiao_nrf52840;
pub mod xiao_stm32c5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardTarget {
    XiaoSense,
    XiaoSensePlus,
    XiaoStm32c5,
}

impl BoardTarget {
    pub const fn name(&self) -> &'static str {
        match self {
            Self::XiaoSense => "xiao-sense",
            Self::XiaoSensePlus => "xiao-sense-plus",
            Self::XiaoStm32c5 => "xiao-stm32c5",
        }
    }

    pub const fn target_triple(&self) -> &'static str {
        match self {
            Self::XiaoSense | Self::XiaoSensePlus => xiao_nrf52840::TARGET_TRIPLE,
            Self::XiaoStm32c5 => xiao_stm32c5::TARGET_TRIPLE,
        }
    }

    pub const fn flash_capacity_bytes(&self) -> u32 {
        match self {
            Self::XiaoSense | Self::XiaoSensePlus => xiao_nrf52840::NRF52840_FLASH_CAPACITY_BYTES,
            Self::XiaoStm32c5 => xiao_stm32c5::STM32C5_FLASH_CAPACITY_BYTES,
        }
    }
}
