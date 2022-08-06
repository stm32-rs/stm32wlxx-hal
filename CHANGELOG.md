# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Changed
- Updated minimum `chrono` version to `0.4.20` to satisfy `cargo-audit`.

## [0.6.1] - 2022-08-01
### Fixed
- Fixed undefined behavior in SPI full duplex RX DMA transfers.

## [0.6.0] - 2022-07-04
### Added
- Added `rcc::Lsco` to use the low-speed oscillator output.
- Added `Rtc::calibrate_lp` to calibrate the RTC.
- Added `Rtc::recalibration_pending`.
- Added `Rtc::is_alarm_{a,b}_en`.
- Added methods to utilize ADC oversampling.

### Changed
- `Rtc.alarm_{a,b}` returns `Alarm` instead of `Option<Alarm>`.
- Updated `stm32-rs` from `0.14.0` to `0.15.1`.

### Fixed
- Fixed a documentation bug in `rtc::Alarm`.  Values are masked if `true`, but the documentation indicated they are masked if `false`.

## [0.5.1] - 2022-05-14
### Added
- Added `Rtc::alarm_{a,b}` to get the alarm value.
- Added `impl From<Alarm> for chrono::NaiveTime`.
- Added `RfSwitch::steal()` to all BSPs.

## [0.5.0] - 2022-05-08
### Added
- Added `set_sleep_clock` to GPIO ports to enable and disable clocks during sleep.
- Added `subghz::Timeout::saturating_add`.
- Added `SubGhz.set_rtc_period` and `SubGhz.restart_rtc` methods required to workaround an erratum with `SubGhz.set_rx_duty_cycle`.
- Added `SubGhz.new_no_reset` and `SubGhz.new_with_dma_no_reset` to create a `SubGhz` without resetting the radio.

### Changed
- Changed minimum supported rust version from 1.57 to 1.60.

### Fixed
- Fixed a typo in the `Exti::set_falling_trigger` function name.
- Fixed endianness in `SubGhz.op_error`.

## [0.4.1] - 2022-03-22
### Added
- Implement `embedded_hal::PwmPin` for `LpTim`.

### Changed
- Inlined trivial `Rng` methods.

### Fixed
- Fixed UART `clock_hz` methods returning the wrong frequency.

## [0.4.0] - 2022-01-08
### Added
- Added a `is_pending` method to the `gpio::Exti` trait.
- Added alarm functionality to the RTC.
- Added `Rtc.is_wakeup_timer_en`.
- Added `flash::flash_range`.
- Added `Flash.program_bytes` for safer flash programming.

### Changed
- Replaced `Debug` implementation with `Display` implementation for:
  - `subghz::FskPacketStatus`
  - `subghz::LoRaPacketStatus`
  - `subghz::Stats`
  - `subghz::Status`
- `Flash::standard_program_generic` now checks for overflow.

### Fixed
- Fixed an off-by-one error in `flash::Page::addr_range`.

### Removed
- Removed `util::reset_cycle_count`; this functionality is now in `cortex-m`.

## [0.3.0] - 2021-12-20
### Added
- Added `info::Core::CT` to get the CPU core at compile time.
- Added `info::Core::from_cpuid()` to get the CPU core at runtime.
- Added a `flash` module with erase and program functionality.
- Added `defmt::Format` for all types declared in the BSPs.
- Added `info::uid::PTR`.

### Changed
- Changed minimum rust version from 1.56 to 1.57 for `const_panic`.
- `info::UID64`
  - Moved to `info::Uid64::PTR`.
  - Changed the type from `*const u8` to `*const u32`.
- Moved functions in `info` into the associated structs/enums.
  - Moved `info::uid64` to `info::Uid64::from_device`.
  - Moved `info::uid64_devnum` to `info::Uid64::read_devnum`.
  - Moved `info::package` to `info::Package::from_device`.
  - Moved `info::uid` to `info::Uid::from_device`.
- Added `#[inline]` to `util::new_delay` and `util::reset_cycle_count`.
- Large dependencies are now optional.
  - `embedded-time` is now an optional feature.
    - Changed `I2C::new` to use `u32` instead of `embedded_time::Hertz`.
  - `chrono` is now an optional feature.

## [0.2.1] - 2021-11-20
### Fixed
- Fixed timeouts after calling `SubGhz::set_sleep`.

## [0.2.0] - 2021-11-11
### Added
- Added two board support crates
  - `nucleo-wl55jc-bsp`
  - `lora-e5-bsp`

### Changed
- Crate ownership changed from [tweedegolf] to [stm32-rs].
  - Thank you [tweedegolf] for the initial development effort!
- Sweeping changes throughout the entire crate, nothing is the same.

## [0.1.0] - 2021-03-26
- Initial release by [tweedegolf]

[tweedegolf]: https://github.com/tweedegolf
[stm32-rs]: https://github.com/stm32-rs
[Unreleased]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.6.1...HEAD
[0.6.1]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.6.0...v0.6.1
[0.6.0]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.5.1...v0.6.0
[0.5.1]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.5.0...v0.5.1
[0.5.0]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.4.1...v0.5.0
[0.4.1]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/stm32-rs/stm32wlxx-hal/releases/tag/v0.2.0
[0.1.0]: https://github.com/tweedegolf/stm32wlxx-hal
