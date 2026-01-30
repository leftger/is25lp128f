# is25lp128f

Async no_std driver for the ISSI IS25LP128F 128 Mbit SPI NOR flash.

## Implemented instructions

The driver implements the full Table 6.3 set (instructions requiring WREN) and companion reads. See the [crate documentation](https://docs.rs/is25lp128f) for the opcode → method table. Highlights:

- Read/write/erase (normal, fast read, 4-byte, quad page program), status and configuration registers, SFDP, suspend/resume, deep power-down, software reset.
- Advanced protection: PPB, DYB, password, lock, freeze, gang lock/unlock, Information Rows, AutoBoot and Bank registers.
- Helpers: `wait_done_timeout`, `read_operation_errors`, `BlockProtectionLevel`, `StatusWrite::with_block_protection`, `read_fast_with_dummy`, `read_information_row`.

## Features

- **std** (default): Enable standard library (e.g. for tests).
- **embedded-storage**: Implement `embedded_storage_async::nor_flash::ReadNorFlash` and `NorFlash`, and `embedded_storage::nor_flash::NorFlashError` for the driver error type. Use with `embedded-storage-async` for async storage abstractions.
- **spi-device**: Adapter from `embedded_hal_async::SpiDevice<u8>` to this crate's `Spi` trait. Use when your SPI is a device (shared bus + CS), e.g. [embassy-embedded-hal](https://crates.io/crates/embassy-embedded-hal)'s shared SPI.
- **sfdp**: SFDP header and BFPT parsing (`sfdp::SfdpHeader`, `sfdp::parse_bfpt`) for discovery of sector size, page size, density. Use with `read_sfdp`.

## embedded-storage compatibility

With the `embedded-storage` feature, `Is25lp128f` implements:

- `embedded_storage::nor_flash::ErrorType`
- `embedded_storage_async::nor_flash::ReadNorFlash` (async read, capacity)
- `embedded_storage_async::nor_flash::NorFlash` (async erase, write)

Constants:

- `READ_SIZE`: 1 byte  
- `WRITE_SIZE`: 256 bytes (page size)  
- `ERASE_SIZE`: 4096 bytes (sector size)  
- Capacity: 16 MiB  

You can use the driver with `embedded_storage_async::nor_flash::RmwNorFlashStorage` or other consumers of the NorFlash traits.

## Example: STM32WBA65RI

An example for the STM32WBA65RI using [embassy-stm32](https://crates.io/crates/embassy-stm32) is in `examples/stm32wba65ri.rs`. It shows:

- Implementing the driver’s `Spi` and `HardwareInterface` with embassy SPI and Timer
- Using the driver as a NorFlash via the embedded-storage-async traits

Build and run (no_std, ARM target):

```bash
cargo build --example stm32wba65ri --no-default-features --features embedded-storage --target thumbv8m.main-none-eabihf --release
probe-rs run --chip STM32WBA65RI target/thumbv8m.main-none-eabihf/release/examples/stm32wba65ri
```

Wire the IS25LP128F to SPI1 (adjust pins to your board): SCK, MISO, MOSI, and a GPIO for CS. The example uses PB4, PA15, PB3, and PA12 by default.

## embassy-embedded-hal (shared SPI bus)

[embassy-embedded-hal](https://crates.io/crates/embassy-embedded-hal) provides shared SPI (and I2C) buses so multiple devices can share one peripheral, each with its own CS. To use it with this driver:

1. Enable the **spi-device** feature: `is25lp128f = { version = "...", features = ["spi-device"] }`.
2. Build a shared SPI bus (e.g. `Mutex::new(embassy_stm32::spi::Spi::new(...))`).
3. Create a device for the flash: `embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(spi_bus, flash_cs_pin)`.
4. Wrap it with our adapter: `is25lp128f::spi_device::SpiDeviceAdapter::new(flash_device)`.
5. Build the driver: `Is25lp128f::new(adapter, your_delay)`.

No dependency on embassy-embedded-hal is required in your `Cargo.toml` for this crate; the adapter only needs `embedded_hal_async::SpiDevice<u8>`, which embassy-embedded-hal’s `SpiDevice` implements.

## License

MIT OR Apache-2.0
