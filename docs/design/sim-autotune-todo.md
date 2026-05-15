# Simulator And Auto-Tune TODO

This document captures the intended future work for simulation fidelity and PID auto-tuning.

## Goal

Use the simulator as a credible dry-run environment for real robots, especially for:

- motion/path algorithm validation
- profile-specific behavior checks
- meaningful auto-tuning experiments

The simulator should not be adjusted by guessing random delays or noise terms. It should be driven by measured real-robot behavior.

## Current Understanding

- Phase 1 drive characterization is useful in sim.
- Phase 3 motion tuning is useful in sim.
- Phase 2 velocity PID tuning with the current CHR method is not reliably meaningful.
- The identified velocity-loop plant often looks like:
  - gain near `1`
  - very small dead time
  - feedforward-dominated closed-loop response
- That likely means either:
  - the simulator is still too idealized at the velocity-loop level, or
  - CHR is the wrong tuning method for this control layer, or
  - both.

## Important Concept

`Dead time` means delay before the system noticeably reacts after a command.

This is different from a slow ramp:

- dead time: command changes, nothing happens yet
- time constant / inertia: command changes, response starts immediately but ramps gradually

CHR tuning assumes meaningful dead time. If the measured plant has almost no dead time, CHR is usually the wrong method.

## Do Not Do

- Do not add arbitrary simulated delays just to make CHR produce nicer gains.
- Do not tune simulator parameters by intuition alone.
- Do not treat exact recovery of simulator internal constants as proof that auto-tune is correct.

## Required Next Step: Measure Real Robots

Before changing simulator timing or dynamics further, gather real step-response data.

### Needed Real-Robot Measurements

1. Closed-loop velocity step response
   - command a step in `vx`
   - log `time`, `commanded vx`, `measured vx`
   - repeat on `drumbot`, `packingbot`, `clawbot`, and any default/reference robot

2. If possible, open-loop/raw motor response
   - apply raw motor/power step
   - log wheel/chassis response
   - this separates plant physics from controller behavior

3. Basic timing facts
   - control loop period
   - odometry/encoder update period
   - velocity filtering details
   - whether commands are applied same-cycle or next-cycle

### Desired Logging Output

CSV or equivalent with at least:

- `time`
- `command_vx`
- `measured_vx`
- optionally `command_wz`
- optionally `measured_wz`
- optionally pose / heading

## After Real Data Exists

1. Compare real closed-loop step response vs sim closed-loop step response.
2. Identify what is actually missing:
   - command latency
   - sensor latency
   - filtering delay
   - drivetrain asymmetry
   - deadband / stiction
   - voltage sag / saturation
   - load-dependent drag
3. Update simulator only with effects justified by real data.
4. Re-check whether CHR is appropriate.

## Likely Outcome

One likely result is that Phase 2 should not use CHR at all for these robots.

If the velocity loop is truly low-dead-time and feedforward-dominated, a better option may be:

- IMC/SIMC-style PI tuning
- a different low-dead-time PI method
- or no extra velocity PID tuning in that regime

## Future Engineering Tasks

### Data Collection

- Add a real-robot step-response logger step.
- Export logs in a format that can be compared directly with sim.
- Add a small plotting/comparison script for real vs sim traces.

### Simulator Fidelity

- Calibrate robot profiles against measured traces instead of config guesses.
- Model only the timing/dynamic effects supported by data.
- Keep deterministic no-noise variants for algorithm tests.
- Keep realistic noisy variants for robustness tests.

### Auto-Tune

- Keep current safe rejection behavior when the identified plant is not a valid CHR target.
- Revisit Phase 2 tuning law after real measurements exist.
- Separate:
  - plant identification quality
  - tuning-law suitability
  - simulator fidelity

### Tests

- Tight deterministic algorithm tests should use deterministic sim profiles.
- Robustness tests should use realistic profile configs.
- Trace-based assertions should be preferred over broad endpoint bands where possible.

## Summary

The right path is:

1. measure real robots
2. compare real vs sim responses
3. improve sim from data
4. only then decide how Phase 2 auto-tuning should work

Until then, the current simulator is useful for characterization, motion tuning, and algorithm testing, but it should not be treated as proof that CHR velocity PID tuning is correct for real robots.
