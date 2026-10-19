# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.2] - 2026-10-19

### Changed
- Use common header file

## [1.1.1] - 2026-05-23

### Added
- Features for location announcement. Uses a map to choose track to play based on message from autostop node.

### Changed
- GPIO pin assignments to match new PCB


## [1.1]  - 2026-04-07

### Added

- Loops Track 2 upon receiving a signal on CAN. Stops looping and plays Track 3 when another signal is received (more realistic horn sound).
- Startup feedback: Plays whatever is Audio Track 1 in the SD card (order of insertion into card) on startup as audible feedback that it is working.

### Changed

- Updated to use CAN 1.13.0
- Updated TX/RX pins of DFPlayer (RX doesn't work, try in second iteration to add 1k resistor)

### Removed
- None