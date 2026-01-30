//! Prelude for convenient imports.
//!
//! ```ignore
//! use is25lp128f::prelude::*;
//! ```

pub use crate::{
    Error, FlashEraseSize, Is25lp128f, JedecId, Status, CHIP_SIZE, PAGE_SIZE, SECTOR_SIZE,
};

#[cfg(feature = "spi-device")]
pub use crate::spi_device::SpiDeviceAdapter;
