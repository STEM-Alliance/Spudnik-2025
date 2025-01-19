// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Robot;

public class SwerveSubsystem extends SubsystemBase {
    //All of the parameters for the swerve modules should be declared in Constants.java
    SwerveModule frontLeft = new SwerveModule(SwerveModuleConstants.FL_STEER_ID, SwerveModuleConstants.FL_DRIVE_ID,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FL_OFFSET_RADIANS,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FL_MOTOR_REVERSED);

    SwerveModule frontRight = new SwerveModule(SwerveModuleConstants.FR_STEER_ID, SwerveModuleConstants.FR_DRIVE_ID,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FR_OFFSET_RADIANS,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FR_MOTOR_REVERSED);

    SwerveModule backRight = new SwerveModule(SwerveModuleConstants.BR_STEER_ID, SwerveModuleConstants.BR_DRIVE_ID,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BR_OFFSET_RADIANS,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BR_MOTOR_REVERSED);

    SwerveModule backLeft = new SwerveModule(SwerveModuleConstants.BL_STEER_ID, SwerveModuleConstants.BL_DRIVE_ID,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BL_OFFSET_RADIANS,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BL_MOTOR_REVERSED);

    /*
     * feature for later
     * public enum RotationStyle {
     * Driver,
     * AutoSpeaker,
     * AutoShuttle
     * }
     * 
     * private RotationStyle rotationStyle = RotationStyle.Driver;
     */

    public final Pigeon2 pigeon = new Pigeon2(Constants.SwerveModuleConstants.PIGEON_ID); //initializes the gyro
    private double pigeonSim;

    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    private Field2d field = new Field2d();

    boolean isalliancereset = false;

    // TO DO: Properly set starting pose
    public final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS,
            getRotation2d(),
            getModulePositions(), new Pose2d());

    public SwerveSubsystem() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        //sets the heading when the robot boots up
        setHeading(-90); // TO DO: Change if needed

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setChassisSpeedsAUTO(speeds), // Method that will drive the robot given ROBOT
                                                                        // RELATIVE ChassisSpeeds. Also optionally
                                                                        // outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        PathPlannerConstants.TRANSLATION_PID,
                        PathPlannerConstants.ROTATION_PID),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    @Override
    public void periodic() {

        //Automatically identifies the alliance color
        if (!isalliancereset && DriverStation.getAlliance().isPresent()) {
            Translation2d pospose = getPose().getTranslation();
            odometry.resetPosition(getRotation2d(), getModulePositions(),
                    new Pose2d(pospose, new Rotation2d(FieldConstants.getAlliance() == Alliance.Blue ? 0.0 : Math.PI)));
            isalliancereset = true;
        }

        odometry.update(getRotation2d(), getModulePositions());

        field.setRobotPose(getPose());

        SmartDashboard.putData("Field", field);

        SmartDashboard.putString("Robot Pose",
                getPose().toString());

        SmartDashboard.putNumberArray("SwerveStates", new double[] {
                frontLeft.getModuleState().angle.getDegrees() + 90, -frontLeft.getModuleState().speedMetersPerSecond,
                frontRight.getModuleState().angle.getDegrees() + 90, -frontRight.getModuleState().speedMetersPerSecond,
                backLeft.getModuleState().angle.getDegrees() + 90, -backLeft.getModuleState().speedMetersPerSecond,
                backRight.getModuleState().angle.getDegrees() + 90, -backRight.getModuleState().speedMetersPerSecond
        });
    }
    /**
     * Sets the heading to the side of the field the robot is currently facing
     */
    public void zeroHeading() {
        setHeading(0);
    }

    /**
     * 
     * @param deg
     */
    public void setHeading(double deg) {
        if (Robot.isSimulation()) {
            pigeonSim = Units.degreesToRadians(deg);
        }
 
        double error = deg - pigeon.getAngle(); //in 2026, getAngle will be removed so look into getYaw()
        double new_adjustment = pigeon.getAngle() + error;
        pigeon.setYaw(new_adjustment);
    }

    public Pose2d getPose() {
        Pose2d p = odometry.getEstimatedPosition();
        return p;
    }

    public void resetOdometry(Pose2d pose) {
        // TO DO: TEST
        setHeading(Units.radiansToDegrees(pose.getRotation().times(-1.0).getRadians()
                + (FieldConstants.getAlliance() == Alliance.Red ? Math.PI : 0.0)));

        SmartDashboard.putNumber("HEading reset to", getHeading());
        SmartDashboard.putBoolean("HASBEENREET", true);
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public double getHeading() {
        return Robot.isSimulation() ? -pigeonSim : Units.degreesToRadians(Math.IEEEremainder(pigeon.getAngle(), 360));
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getHeading());
    }

    public void stopDrive() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState[] states) {
        lastChassisSpeeds = DriveConstants.KINEMATICS.toChassisSpeeds(states);
        // Normalize speeds so they are all obtainable
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_MODULE_VELOCITY);
        frontLeft.setModuleState(states[Constants.DriveConstants.ModuleIndices.FRONT_LEFT]);
        frontRight.setModuleState(states[Constants.DriveConstants.ModuleIndices.FRONT_RIGHT]);
        backRight.setModuleState(states[Constants.DriveConstants.ModuleIndices.REAR_RIGHT]);
        backLeft.setModuleState(states[Constants.DriveConstants.ModuleIndices.REAR_LEFT]);
    }

    public void setChassisSpeedsAUTO(ChassisSpeeds speeds) {
        double tmp = speeds.vxMetersPerSecond;
        speeds.vxMetersPerSecond = speeds.vyMetersPerSecond;
        speeds.vyMetersPerSecond = tmp;
        tmp = speeds.omegaRadiansPerSecond;
        speeds.omegaRadiansPerSecond *= -1;
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModules(states);
    }

    /**
     * Sets the wheel position to form an X so the robot can't be pushed
     * Helpful for resisting defense while place game pieces
     */
    public void setXstance() {
        frontLeft.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
        frontRight.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        backLeft.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        backRight.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = DriveConstants.KINEMATICS.toChassisSpeeds(
                frontLeft.getModuleState(),
                frontRight.getModuleState(),
                backLeft.getModuleState(),
                backRight.getModuleState());

        return Robot.isSimulation() ? lastChassisSpeeds : speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = {
                frontLeft.getModulePosition(),
                frontRight.getModulePosition(),
                backLeft.getModulePosition(),
                backRight.getModulePosition()
        };

        return states;
    }

    @Override
    public void simulationPeriodic() {
        frontLeft.simulate_step();
        frontRight.simulate_step();
        backLeft.simulate_step();
        backRight.simulate_step();
        pigeonSim += 0.02 * lastChassisSpeeds.omegaRadiansPerSecond;
    }

    /*
     * feature for later
     * public RotationStyle getRotationStyle() {
     * return rotationStyle;
     * }
     * 
     * public void setRotationStyle(RotationStyle style) {
     * rotationStyle = style;
     * }
     */

    /*
     * // ---------- Path Planner Methods ---------- \\
     * 
     * public Command loadPath(String name) {
     * return new PathPlannerAuto(name);
     * }
     * 
     * public Command followPathCommand(String pathName) {
     * PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
     * 
     * return new FollowPathHolonomic(
     * path,
     * this::getPose, // Robot pose supplier
     * this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
     * this::setChassisSpeedsAUTO, // Method that will drive the robot given ROBOT
     * RELATIVE ChassisSpeeds
     * new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
     * likely live in your
     * // Constants class
     * PathPlannerConstants.TRANSLATION_PID, // Translation PID constants
     * PathPlannerConstants.ROTATION_PID, // Rotation PID constants
     * DriveConstants.MAX_MODULE_VELOCITY, // Max module speed, in m/s
     * DriveConstants.DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance
     * from robot center to
     * // furthest module.
     * new ReplanningConfig() // Default path replanning config. See the API for the
     * options here
     * ),
     * () -> {
     * // Boolean supplier that controls when the path will be mirrored for the red
     * // alliance
     * // This will flip the path being followed to the red side of the field.
     * // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
     * 
     * var alliance = DriverStation.getAlliance();
     * if (alliance.isPresent()) {
     * return alliance.get() == DriverStation.Alliance.Red;
     * }
     * return false;
     * },
     * this // Reference to this subsystem to set requirements
     * );
     * }
     * 
     * public PathPlannerPath generateOTFPath(Translation2d... pathPoints) {
     * // Create the path using the bezier points created above
     * PathPlannerPath path = new PathPlannerPath(
     * List.of(pathPoints),
     * new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints
     * for this path. If using a
     * // differential drivetrain, the angular
     * // constraints have no effect.
     * new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can
     * set a holonomic rotation
     * // here. If using a differential drivetrain, the
     * // rotation will have no effect.
     * );
     * 
     * // Prevent the path from being flipped if the coordinates are already correct
     * path.preventFlipping = true;
     * 
     * return path;
     * }
     */
}