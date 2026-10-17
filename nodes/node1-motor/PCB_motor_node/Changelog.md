# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.1.1] - 17-10-26 - Nhan Nguyen
## Changed
- Use common header file

## [2.1.0] - 28-04-26 - Nhan Nguyen
## Added
- Dual serial module

## [2.0.2] - 20-04-26 - Chiara Gillam
### Changed
- autostop doesnt restart once finished
- can autstop multiple times

## Added 
- Service brake can be applied from mc node


## [2.0.1] - 11-04-26 - Chiara Gillam
### Changed
- allocated more memory for can bus 4096 > 6144

### Added
- added od_service_brake_mc


## [2.0.0] - 09-04-26 - Sean Larkin
### Changed
- AutoStopChallenge completely reworked — throttle now tapers proportionally
  to distance remaining over 25m rather than PI speed control, cutting to zero
  at 25m then applying full brake DAC until stopped
- EnergyRecoveryChallenge completely reworked — replaced regen-first strategy
  with four-phase state machine (waiting, braking, idle, motoring)
- Challenge mode values updated to match driver controls node
  (1=Throttle, 2=Speed, 3=AutoStop, 4=EnergyRecovery, 5=Traction)
- od_motor_command type corrected to uint16_t (0–1023)
- od_motor_command scaling updated from 255 to 1023 across SpeedControl,
  TractionChallenge, and ThrottleControl
- ThrottleControl scaling corrected — direct 1:1 mapping replaces erroneous
  left-shift
- ReadSpeedKMH now uses elapsed time from previous_millis for accurate speed
  calculation
- Speed cap check moved before challenge dispatch in OperationalMode
- PI integrator time constant updated to use LOOP_INTERVAL_MS for correctness
- od_direction_mode values corrected to match OD spec (1=back, 2=neutral, 3=fwd)

### Added
- od_recovered_energy: new OD variable receiving recovered energy from battery
  node (0x2000:04)
- RPDO1 configured to receive recovered_energy_can from battery node 7 TPDO2
- EnergyRecoveryChallenge phase state machine:
  - PHASE_WAITING_FOR_BRAKE: throttle control while waiting for first brake input
  - PHASE_BRAKING: regen braking with service brake suppressed, snapshots energy
    on entry and exit
  - PHASE_STOPPED_IDLE: holds service brake, waits for first throttle input
  - PHASE_MOTORING: raw throttle pass-through, cuts throttle when energy budget
    exhausted
  - PHASE_BUDGET_EXHAUSTED: all outputs cut, challenge complete
- AutoStopChallenge distance tracking using accumulated PCNT pulses via
  total_pulse_accum global, avoiding counter clear conflict with ReadSpeedKMH
- total_pulse_accum global for cross-function distance accumulation
- ISOLATING_RELAY pinMode initialised in setup
- od_recovered_energy OD registration in setup

## [1.1.0] - 02-04-26 - Sean Larkin
### Changed
- Renamed all OD variables to comply with `od_` prefix and snake_case standards
- Renamed all global variables to snake_case
- Renamed all functions to UpperCamelCase
- Renamed all constants to UPPER_CASE and moved to header file
- Replaced LEDC PWM output with I2C DAC (WriteDAC)
- ReadSpeedKMH now takes prev_time argument for accurate elapsed-time speed
  calculation

### Added
- SpeedControl: PI closed-loop speed controller
- ThrottleControl: Raw motor command pass-through to DAC
- AutoStopChallenge: PI speed control with automatic stop below threshold
- TractionChallenge: Speed control with wheel slip detection and throttle cut
- EnergyRecoveryChallenge: Regen-first braking strategy suppressing service brake
- Challenge mode dispatch in OperationalMode via od_challenge_mode
- Hard speed cap at 15 km/h cutting throttle universally across all modes
- Function prototypes and all pin/constant definitions moved to motor_PCB.h

## [1.0.1] - 14-03-26
### Changed
- Changed the common config to align with what is in the datasheet

## [1.0.0] - 14-03-26
### Added
- Created the first version of PCB_motor_node