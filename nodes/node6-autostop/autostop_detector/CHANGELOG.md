# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.3] - 2026-10-19
### Changed
  Use common header file

## [1.1.2] - 2026-05-30
### Added
  Location announcement SDO automatically hard-sets itself in certain challenge modes (eg. autostop, traction)

## [1.1.1] - 2026-05-27
### Changed
  Location announcement SDO properly increments now.

## [1.1.0] - 2026-05-19
### Added 
  Added train location announcement system using od_location_counter (OD 0x1051:00).
  Implemented location-based audio trigger handling for passing and standstill announcements.
  Added _HandleLocationAnnoucement() state handler for automatic announcement dispatch.
  Added _ChangeLocation() marker detection routine to increment train location state.
  Added _SendPassing() helper function to transmit passing announcements to Audio Node.
  Added _SendStandStill() helper function to transmit standstill announcements when train speed is zero.
  Added OD variable:
    od_location_counter (0x1051:00) — current train location index.
  Added OD variable:
    od_true_speed (0x606C:00) — true motor speed received via RPDO1.
    
