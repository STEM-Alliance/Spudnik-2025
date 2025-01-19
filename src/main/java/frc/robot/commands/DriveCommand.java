// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.SwerveSubsystem.RotationStyle;
// import org.opencv.core.RotatedRect;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.controller.ProfiledPIDController;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    // Auto rotation pid/rate limiter
    private PIDController rotationController = new PIDController(6, 0.0, 0.5);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;

    private enum DriveState {
        Free,
        Locked,
    };

    private DriveState state = DriveState.Free;

    public DriveCommand(SwerveSubsystem swerveSubsystem, XboxController xbox) {
        this.swerveSubsystem = swerveSubsystem;
        this.xbox = xbox;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        dsratelimiter.reset(SLOWMODE_MULT);

        addRequirements(swerveSubsystem);
    }

    //returns the value in a range
    double clamp(double v, double min, double max) {
        // v = value entereds
        return (v < min) ? min : (v > max ? max : v);
    }

    //Fancy deadzone (so you can still approach the max value)
    //For a translational input (x- and y-axis)
    public Translation2d DeadBand(Translation2d input, double deadzone) {
        double mag = input.getNorm();
        Translation2d norm = input.div(mag);

        if (mag < deadzone) {
            return new Translation2d(0.0, 0.0);
        } else {
            Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
            return new Translation2d(
                    clamp(result.getX(), -1.0, 1.0),
                    clamp(result.getY(), -1.0, 1.0));
        }
    }

    //Fancy deadband for a single number input (Z-axis)
    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {
        double xSpeed, ySpeed, zSpeed;
        Translation2d xyRaw;

        xyRaw = new Translation2d(xbox.getLeftX(), xbox.getLeftY());
        Translation2d xySpeed = DeadBand(xyRaw, 0.5 / 2.f);
        zSpeed = DeadBand(xbox.getRightX(), 0.25);
        xSpeed = -xySpeed.getX();
        ySpeed = xySpeed.getY();

        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        //rate limiter so you can accelerate fast without brownouts
        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * 1 + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;

        // Zero heading: if code doesn't work to automatically zero heading
        // if (xbox.getXButton()) {
        //     swerveSubsystem.zeroHeading();
        //     Translation2d pospose = swerveSubsystem.getPose().getTranslation();
        //     swerveSubsystem.odometry.resetPosition(swerveSubsystem.getRotation2d(),
        //             swerveSubsystem.getModulePositions(),
        //             new Pose2d(pospose, new Rotation2d(FieldConstants.getAlliance() == Alliance.Blue ? 0.0 : Math.PI)));
        // }

        ChassisSpeeds speeds;

        // Drive Non Field Oriented
        if (!xbox.getRawButton(5)) {
            
             speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed, zSpeed,
                    new Rotation2d(
                           -swerveSubsystem.getRotation2d().rotateBy(DriveConstants.PIGEON_ANGLE_OFFSET).getRadians()));
        } else if (xbox.getRawButton(5)) {
           speeds = new ChassisSpeeds(-xSpeed, -ySpeed, zSpeed);
        } else {
            // Normal non-field oriented
            speeds = new ChassisSpeeds(-xSpeed, -ySpeed, zSpeed);
        }

        SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        swerveSubsystem.setModules(calculatedModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

    public boolean isFinished() {
        return false;
    }
  
}
