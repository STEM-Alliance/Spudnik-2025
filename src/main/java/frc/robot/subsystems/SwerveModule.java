// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Robot;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private double driveEncSim = 0;
    private double steerEncSim = 0;

    private final com.ctre.phoenix6.hardware.CANcoder absoluteEncoder;

    private final double motorOffsetRadians;
    private final boolean isAbsoluteEncoderReversed;
    private final boolean motor_inv;

    private final PIDController steerPID;

    private static int moduleNumber = 0;
    int thisModuleNumber;

    SlewRateLimiter turnratelimiter = new SlewRateLimiter(4.d); 

    public SwerveModule(int steerCanID, int driveCanID, int absoluteEncoderPort, double motorOffsetRadians,
            boolean isAbsoluteEncoderReversed, boolean motorReversed) {
        driveMotor = new SparkMax(driveCanID, MotorType.kBrushless);
        SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
        driveMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        driveMotorConfig.inverted(false); //NOTE: this was true before

        steerMotor = new SparkMax(steerCanID, MotorType.kBrushless);
        SparkMaxConfig steerMotorConfig = new SparkMaxConfig();
        steerMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        steerMotorConfig.inverted(false); 

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); //new for 2025
        steerMotor.configure(steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); //new for 2025

        this.motor_inv = motorReversed;
        driveMotorEncoder = driveMotor.getEncoder();
        steerMotorEncoder = steerMotor.getEncoder();

        // TO DO: Reset encoder offsets possibly set in Tuner X
        absoluteEncoder = new com.ctre.phoenix6.hardware.CANcoder(absoluteEncoderPort);
        com.ctre.phoenix6.configs.CANcoderConfiguration cfg = new com.ctre.phoenix6.configs.CANcoderConfiguration();
        cfg.MagnetSensor = new com.ctre.phoenix6.configs.MagnetSensorConfigs();
        cfg.MagnetSensor.MagnetOffset = 0.0f;
        absoluteEncoder.getConfigurator().apply(cfg);
        // CANcoderConfigurator configurator = absoluteEncoder.getConfigurator();

        this.motorOffsetRadians = motorOffsetRadians;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        steerPID = new PIDController(SwerveModuleConstants.MODULE_KP, 0, SwerveModuleConstants.MODULE_KD);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        thisModuleNumber = moduleNumber;
        moduleNumber++;
        
        resetEncoders();
    }

    /**
     * Updates the siulation encoder
     */
    public void simulate_step() {
        driveEncSim += 0.02 * driveMotor.get() * (DriveConstants.MAX_MODULE_VELOCITY);
        steerEncSim += 0.02 * steerMotor.get() * (10.0);
    }

    /**
     * Returns drive poisiton in meters
     * @return
     */
    public double getDrivePosition() {
        if (Robot.isSimulation())
            return driveEncSim;
        return driveMotorEncoder.getPosition()*SwerveModuleConstants.DRIVE_ROTATION_TO_METER;
    }

    /**
     * Returns drive velocity in meters per minute
     * @return
     */
    public double getDriveVelocity() {
        return driveMotorEncoder.getVelocity()*SwerveModuleConstants.DRIVE_METERS_PER_MINUTE;
    }

    /**
     * Returns steer position in radians
     * @return
     */
    public double getSteerPosition() {
        if (Robot.isSimulation())
            return steerEncSim;
        return steerMotorEncoder.getPosition()*SwerveModuleConstants.STEER_ROTATION_TO_RADIANS;
    }

    /**
     * Returns steer velocity in Radians per minute
     * @return
     */
    public double getSteerVelocity() {
        return steerMotorEncoder.getVelocity()*SwerveModuleConstants.STEER_RADIANS_PER_MINUTE;
    }

    public double getAbsoluteEncoderPosition() {
        double angle = Units.rotationsToRadians(absoluteEncoder.getPosition().getValueAsDouble());// * (Math.PI /
        // 180.d);
        angle -= motorOffsetRadians;
        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Drive motors are set to position zero
     * Steer motors set to the absolute encoder position
     */
    public void resetEncoders() {
        driveMotorEncoder.setPosition(0);
        steerMotorEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(-getSteerPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(),
                new Rotation2d(-getSteerPosition()).rotateBy(DriveConstants.PIGEON_ANGLE_OFFSET.times(-1)));
    }

    public void setModuleStateRaw(SwerveModuleState state) {
        //in 2026, use the instance method instead of optimize which will be removed
        state = SwerveModuleState.optimize(state, new Rotation2d(getSteerPosition()));
        double drive_command = state.speedMetersPerSecond / DriveConstants.MAX_MODULE_VELOCITY;
        driveMotor.set(drive_command * (motor_inv ? -1.0 : 1.0));

        
        double steercmd = steerPID.calculate(getSteerPosition(), state.angle.getRadians());
        if (Robot.isSimulation()) {
            steerMotor.set(steercmd);
        } else {
            steerMotor.setVoltage(12 * steercmd);
        }
        
        SmartDashboard.putNumber("Drive" + thisModuleNumber, drive_command);
    }

    //Slight drive to keep the wheels in place. Otherwise WPILib will default to moving the wheels straight
    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        setModuleStateRaw(state);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}