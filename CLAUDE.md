# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FTC (FIRST Tech Challenge) robotics codebase for team DECODE 2025-2026 season. Ring shooter robot with turret auto-aim, mecanum drive, and Road Runner trajectory planning.

## Build Commands

```bash
./gradlew build              # Build all modules
./gradlew :TeamCode:build    # Build only TeamCode
./gradlew clean              # Clean build artifacts
```

This is an Android Gradle project (AGP 8.7.0). Builds are typically done via Android Studio and deployed to the REV Control Hub over ADB/WiFi Direct. There are no unit tests — verification happens on the physical robot.

## Architecture

All team code lives under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

### Key Packages

- **OpModes/** — Runnable programs registered with the FTC SDK. `AutomBlue` and `AutomRed` are autonomous modes (mirrored by Y-axis). `TzeleOp` is the driver-controlled teleop.
- **classes/** — Hardware subsystems (`Intake`, `Outtake`). Each wraps motors/servos and exposes an `update(Gamepad, ...)` method called every loop.
- **drive/** — Road Runner mecanum drive implementation. `SampleMecanumDrive` is the main drivetrain class; `DriveConstants` holds tuning parameters (GoBilda 5203 motors, 383.6 CPR, 435 RPM). `StandardTrackingWheelLocalizer` provides dead-reckoning via tracking wheels.
- **trajectorysequence/** — Road Runner trajectory sequencing (trajectories + waits + turns).
- **util/** — `PoseStorage` persists robot pose and alliance color across auto→teleop transitions. Other utilities for logging, encoders, and dashboard.

### Data Flow

1. **Autonomous** (`AutomBlue`/`AutomRed`): Builds Road Runner trajectory sequences, runs them with `followTrajectorySequenceAsync()`, and calls subsystem methods between segments. Saves final pose to `PoseStorage`.
2. **TeleOp** (`TzeleOp`): Reads `PoseStorage` for initial pose. Main loop calls `drive.update()` then `intake.update()` and `outtake.update()` each cycle.
3. **Outtake auto-aim**: When enabled, calculates turret angle from `atan2(dy, dx)` relative to robot heading, converts to encoder ticks, and sets turret target. Shooter velocity is distance-based.

### Hardware Map

- Drive motors: `leftFront`, `leftRear`, `rightRear`, `rightFront`
- Turret motor: `tureta` (RUN_TO_POSITION mode, external gear ratio 4.7:1)
- Shooter motors: `shooter`, `shooter2` (dual, reversed, PIDF velocity control)
- Intake motor: `intake` (GoBilda 1100 RPM)
- Servos: `servoIntake` (arm), `servoUnghi` (hood angle)

## Key Libraries

- **Road Runner 0.5.6** — Trajectory generation, spline paths, async following
- **FTC SDK 11.1.0** — Hardware abstraction, OpMode lifecycle
- **FTC Dashboard 0.5.1** — Real-time telemetry over WiFi (accessible at 192.168.43.1:8080)
- **MeepMeep** — Offline trajectory visualization (separate module, run as desktop Java app)

## Important Constants

- Turret limits: -315 to 1395 encoder ticks
- Turret ticks per revolution: 383.6 * 4.7 = 1802.92
- Shooter PIDF: (0.8, 0, 0.5, 11.7), idle velocity 1100 ticks/s
- Goal coordinates: X=-65, Y=-65 (blue) / Y=65 (red)
- Drive max velocity: 60 in/s, max accel: 60 in/s²
