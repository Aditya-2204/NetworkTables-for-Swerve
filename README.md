# NetworkTablesDrivetrain

**NetworkTablesDrivetrain** is a sample FRC robot project featuring a swerve drivetrain that publishes real-time data to NetworkTables. This repository serves as a base for visualization tools, analysis software, or other projects that rely on live drivetrain telemetry.

## Overview

- Implements a fully functional swerve drive
- Updates drivetrain state—including module angles, speeds, and gyro orientation—to NetworkTables in real-time
- Designed for integration with external tools (e.g. custom dashboards, simulators, or AutoViz)
- Compatible with WPILib and standard FRC robot architecture

## Features

- ✅ Real-time NetworkTables updates
- 🔁 Swerve module position and velocity tracking
- 🧭 Gyroscope (e.g. Pigeon2) integration
- 🛠 Easily modifiable for other drive types

## Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/Aditya-2204/NetworkTablesDrivetrain.git
   ```
2. Open the project in Visual Studio Code with WPILib extension
3. Deploy to your roboRIO or simulate as needed.
