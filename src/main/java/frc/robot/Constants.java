// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/**
 * Only declare variables that will not change while the code is running in this class (i.e. CAN IDs)
 * Variables should be declared as public static final and good practice is to use all capital letters for final variables
 * 
 * Access these variables in other files by typing the className.variableName (i.e. SwerveModuleConstants. FL_DRIVE_ID)
 */
public final class Constants {
  public static class OperatorConstants {
    //these are the ports of the Driver Station each xbox controller should be assigned to
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int AUX_CONTROLLER_PORT = 1;
  }

  public static final class FieldConstants {
    public static final double GRAVITY = 9.81; 

    public static Alliance getAlliance() {
      if (DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get();
      }

      return Alliance.Blue;
    }
  }

  public static class SwerveModuleConstants {
    public static final int PIGEON_ID = 4; //TO DO

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEERING_GEAR_RATIO = 1 / 12.8; //TO DO: Ask Ian
    // This is for L1 SDS Modules
    public static final double DRIVE_GEAR_RATIO = 8.14; //TO Do: Ask Ian

    public static final double DRIVE_ROTATION_TO_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double STEER_ROTATION_TO_RADIANS = STEERING_GEAR_RATIO * Math.PI * 2d;
    public static final double DRIVE_METERS_PER_MINUTE = DRIVE_ROTATION_TO_METER / 60d;
    public static final double STEER_RADIANS_PER_MINUTE = STEER_ROTATION_TO_RADIANS / 60d;

    // TO DO: Tune for modules!
    public static final double MODULE_KP = 0.3048; //TO DO
    public static final double MODULE_KD = 0.0066806; //TO DO

    // TO DO: change the reversed to true if needed for each module
    // --------- Front Left Module --------- \\
    public static final int FL_DRIVE_ID = 2;
    public static final int FL_STEER_ID = 3;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 1; //TO DO
    public static final double FL_OFFSET_RADIANS = Units.rotationsToRadians(-0.322998) + Math.PI / 2; //TO DO: find offset
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FL_MOTOR_REVERSED = false;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 6;
    public static final int FR_STEER_ID = 7;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 5; //TO DO
    public static final double FR_OFFSET_RADIANS = Units.rotationsToRadians(0.445557) + Math.PI / 2; //TO DO: find offset
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FR_MOTOR_REVERSED = false;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 11;
    public static final int BR_STEER_ID = 12;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 13; //TO DO
    public static final double BR_OFFSET_RADIANS = Units.rotationsToRadians(-0.09) + Math.PI / 2; //TO DO: find offset
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BR_MOTOR_REVERSED = false;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 9;
    public static final int BL_STEER_ID = 10;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 8; //TO DO
    // -0.401123
    public static final double BL_OFFSET_RADIANS = Units.rotationsToRadians(-0.401123) + Math.PI / 2; //TO DO: find offset
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BL_MOTOR_REVERSED = false; //NOTE: this was true before

  }

  public static class DriveConstants {
    // TO DO: Update max vel 
    public static final double MAX_MODULE_VELOCITY = 5.21; //TO DO: Measure
    public static final double MAX_ROBOT_VELOCITY = 5.21; //TO DO: Measure
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0; // Approx. Measured rads/sec

    //Updated based on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(15); //Distance between center of the wheel to center of wheel
    public static final double WHEEL_BASE = Units.inchesToMeters(15); 
    // TO DO: Set angle offset relative to front of robot
    public static final Rotation2d PIGEON_ANGLE_OFFSET = Rotation2d.fromDegrees(0); //doesn't matter if pigeon is not rotated
    // TO DO: For PPLib, max radius of drivebase
    public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(15); //for old pathplanner

    //Changes which modules get what commands
    public static final class ModuleIndices {
      public static final int FRONT_LEFT = 3; //TO DO
      public static final int FRONT_RIGHT = 2; //TO DO
      public static final int REAR_LEFT = 1;//TO DO
      public static final int REAR_RIGHT = 0; //TO DO
    }

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0));

    public static final double XY_SPEED_LIMIT = 0.5; //TO DO: Change after inital testing
    public static final double Z_SPEED_LIMIT = 0.5; //TO DO: Change after inital testing
  }
  public static final class PathPlannerConstants {
    //TO DO: Change based on tuning
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0); //could put d to 0.2
    public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0);
  }

  public static final class PoseConstants {

    //was for Limelight, could use with swerve drive pose estimator if tuned
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevTheta = 500;
  }
}
