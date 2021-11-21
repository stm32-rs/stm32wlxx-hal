# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Added
- Added `info::CORE` to get the CPU core at compile time.
- Added `info::core()` to get the CPU core at runtime.
- Added a `flash` module with erase and program functionality.

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
[Unreleased]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.2.1...HEAD
[0.2.1]: https://github.com/stm32-rs/stm32wlxx-hal/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/stm32-rs/stm32wlxx-hal/releases/tag/v0.2.0
[0.1.0]: https://github.com/tweedegolf/stm32wlxx-hal
