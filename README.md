# 2025 Code

## Comments that start with TO DO indicate a value that needs to be changed or verified

## Setup

- Install [WPILib Suite](https://github.com/wpilibsuite/allwpilib/releases/latest), following the included instructions.
  - VS Code, WPILib Code APIs, SmartDashboard, and other tools
  - Internet is required to first download the Vendor libraries (CTRE/NavX)
- Install [NI FRC Update Suite](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html), following the included instructions.
  - FRC Driver Station

## Install Third Party Librariers
  
1. In VSCode: `CTRL+SHIFT+P` 
2. then type `WPILib: Manage`
3. then go down to `Install New librariers (online)`
4. Paste the following into the dialog box:
This link should be for the Pigeons and SparkMax
`https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json`
`https://software-metadata.revrobotics.com/REVLib-2025.json`

PathPlanner lib:
`https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json`

Repeat steps 1 through 3 then paste:

`https://software-metadata.revrobotics.com/REVLib-2025.json`

## Configure Spark Max Controllers

The SPARK MAX Controllers have a really nice config app and documentation. The documentation can be found here: https://docs.revrobotics.com/sparkmax/gs-sm

The hardware client app can be found here: https://docs.revrobotics.com/sparkmax/rev-hardware-client/getting-started-with-the-rev-hardware-client

Connect a USB cable from your PC to the SPARK Max controller. The app will detect this and display the controller. From the app you can program firmware, configure the motor, and configure the CAN ID. **This must be unique from all controllers on the bus.**

## Commands

- The VSCode Build Menu: `CTRL+SHFT+P`
  - Type `WPILib: Build Robot Code`
- VSCode Command Palette: `CTRL+SHFT+P`
  - Auto-fills on commands
- Deploy to the RoboRIO
  - `CTRL+SHIFT+P`
  - Type: `WPILib: Deploy Robot Code`

# Motor Current Limits
ALL motor will have current limits setup. Below is a table of the current limits that are deemed good:

```
  static public int NeoLimit = 80;
  static public int Neo550Limit = 20;
  static public int BagMotorLimit = 30; // Max power is 149 W, 12.4 A
  static public int M775ProLimit = 15; // Max power 347 W, 28.9 A
  // https://firstwiki.github.io/wiki/denso-window-motor
  static public int WindowLimit = 15; // This seems safe
```

# 2025 Controls
## Drive Controller (Port 0)
*A = 
*B = 
*X = 
*Y = 
*Left bumper = 
*Right bumper = 
*Left trigger = 
*Right rigger = 
*Left stick = translate
*Right stick = rotate
*POV up = 
*POV down = 
*POV left = 
*POV right = 

## Aux Controller (Port 1)
*A = 
*B = 
*X = 
*Y = 
*Left bumper = 
*Right bumper = 
*Left trigger = 
*Right trigger = 
*Left stick = 
*Right stick = 
*POV up = 
*POV down = 
*POV left = 
*POV right = 
