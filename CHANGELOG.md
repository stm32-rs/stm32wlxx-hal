# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Added
- Added a `is_pending` method to the `gpio::Exti` trait.
- Added alarm functionality to the RTC.

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
[Unreleased]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/stm32-rs/stm32wlxx-hal/releases/tag/v0.2.0
[0.1.0]: https://github.com/tweedegolf/stm32wlxx-hal
