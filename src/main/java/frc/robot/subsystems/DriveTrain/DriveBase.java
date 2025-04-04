// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.subsystems.DriveTrain.SwerveModules.DevBotConstants;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule;
import frc.util.FastGyros.GyroIO;
import frc.util.FastGyros.NavxIO;
import frc.util.FastGyros.PigeonIO;
import frc.util.FastGyros.SimGyroIO;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.util.function.DoubleSupplier;

/**
 * The DriveBase class represents the drivetrain of the robot.
 * It controls the movement and positioning of the robot using swerve drive.
 */
@SuppressWarnings("unused")
public class DriveBase extends SubsystemBase {
    /**
     * the array of the modules themselves
     */
    private final SwerveModule[] modules = new SwerveModule[4];

    /**
     * the kinematics of the swerve drivetrain (how the modules are placed and calculating the kinematics)
     */
    private final SwerveDriveKinematics driveTrainKinematics = new SwerveDriveKinematics(DriveBaseConstants.MODULE_TRANSLATIONS);

    /**
     * a variable representing the deltas of the modules (how much they have moved since the last update)
     */
    private final SwerveModulePosition[] moduleDeltas =
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    /**
     * the pose estimator of the drive base (takes the odometry (which is wheel distance and gyro)
     * and calculates the pose of the robot and also can take vision measurements)
     */
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(driveTrainKinematics, new Rotation2d(), moduleDeltas, new Pose2d());

    /**
     * the gyro of the robot. it can be navx or pigeon 2
     */
    private final GyroIO gyro;

    /**
     * the inputs of the drive base (all the motor voltages, angles, speeds, etc.) to be logged
     */
    private final DriveBaseInputsAutoLogged inputs = new DriveBaseInputsAutoLogged();

    /**
     * the max speed the wheels can spin (drive motor at max speed)
     */
    public final double MAX_FREE_WHEEL_SPEED = DevBotConstants.MAX_WHEEL_LINEAR_VELOCITY;

    /**
     * the target states of the modules (the states the modules should be in)
     */
    private SwerveModuleState[] targetStates =
            new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};


    /**
     * the current pose of the robot (the position and rotation of the robot)
     */
    private Pose2d currentPose = new Pose2d();

    @AutoLog
    public static class DriveBaseInputs {
        protected double XspeedInput = 0; //the X speed input
        protected double YspeedInput = 0; //the Y speed input

        protected double rotationSpeedInput = 0;

        protected SwerveModuleState[] currentStates =
                new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()}; //the current states of the modules

        protected String activeCommand; //the active command of the robot
    }

    /**
     * Creates a new DriveBase.
     */
    public DriveBase() {
        modules[0] = new SwerveModule(SwerveModule.ModuleName.Front_Left);
        modules[1] = new SwerveModule(SwerveModule.ModuleName.Front_Right);
        modules[2] = new SwerveModule(SwerveModule.ModuleName.Back_Left);
        modules[3] = new SwerveModule(SwerveModule.ModuleName.Back_Right);

        if (RobotBase.isReal()) {
            gyro = switch (Constants.ROBOT_TYPE) {
                case DEVELOPMENT, COMPETITION -> new PigeonIO(DriveBaseConstants.PIGEON_ID);
                case REPLAY -> throw new IllegalArgumentException("Robot cannot be replay if it's real");
            };
            gyro.reset(new Rotation2d());
        } else gyro = new SimGyroIO(() -> driveTrainKinematics.toTwist2d(moduleDeltas), this::getChassisSpeeds);

        for (int i = 0; i < 4; i++) modules[i].resetDriveEncoder();

        SmartDashboard.putData("Gyro", gyro);

        RobotConfig config = DriveBaseConstants.PathPlanner.ROBOT_CONFIG;

        PathFollowingController pathPlannerPIDController = new PPHolonomicDriveController(
                DriveBaseConstants.PathPlanner.XY_PID.createPIDConstants(),
                DriveBaseConstants.PathPlanner.THETA_PID.createPIDConstants()
        );

        AutoBuilder.configure(
                this::getPose,
                this::reset,
                this::getChassisSpeeds,
                this::drivePP,
                pathPlannerPIDController,
                config,
                () -> DriverStation.getAlliance().isPresent() &&
                            DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this);

                setModulesBrakeMode(true);
    }


    /**
     * resets the robot to 0, 0 and a rotation of 0 (towards red alliance)
     */
    public Command resetOnlyDirection() {
        return new InstantCommand(() -> {
            Pose2d currentPose = this.getPose();

            if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
                currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(Math.PI));
            else
                currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d());

            SwerveModulePosition[] positions = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) positions[i] = modules[i].modulePeriodic();

            gyro.reset(currentPose.getRotation());

            poseEstimator.resetPosition(currentPose.getRotation(), positions, currentPose);

        }).withName("Reset Only Direction").ignoringDisable(true);
    }

    public void setModulesBrakeMode(boolean isBrake) {
        for (int i = 0; i < 4; i++) {
            modules[i].setIdleMode(isBrake);
        }
    }

    /**
     * resets the robot to a new pose
     *
     * @param newPose the new pose the robot should be in
     */
    public void reset(Pose2d newPose) {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].modulePeriodic();

        poseEstimator.resetPosition(newPose.getRotation(), positions, newPose);

        gyro.reset(newPose.getRotation());
        currentPose = newPose;
        Logger.processInputs(getName(), inputs);
    }

    /**
     * gets the acceleration of the robot in the X direction
     *
     * @return the acceleration of the robot in the X direction m/s^2
     */
    public double getXAcceleration() {
        return gyro.getAccelerationX();
    }

    /**
     * gets the acceleration of the robot in the Y direction
     *
     * @return the acceleration of the robot in the Y direction m/s^2
     */
    public double getYAcceleration() {
        return gyro.getAccelerationY();
    }

    /**
     * returns the reported Rotation by the Navx
     *
     * @return Rotation2d reported by the Navx
     */
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * returns the current angle of the robot
     *
     * @return the angle of the robot (left is positive) IN DEGREES
     */
    public double getAngle() {
        return -gyro.getYaw();
    }

    public double getVelocity(){
        ChassisSpeeds speed = getChassisSpeeds();

        double totalSpeed = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);

        Logger.recordOutput("DriveBase/velocity", totalSpeed);

        return totalSpeed;
    }

    /**
     * gets the current chassisSpeeds of the robot
     *
     * @return the current chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return driveTrainKinematics.toChassisSpeeds(inputs.currentStates);
    }

    /**
     * gets the current absolute (field relative) speeds of the robot
     *
     * @return the current chassis speeds
     */
    public ChassisSpeeds getAbsoluteChassisSpeeds() {
        ChassisSpeeds speeds = getChassisSpeeds();

        return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation2d());
    }

    /**
     * gets the current pose of the robot
     *
     * @return the current pose of the robot
     */
    public Pose2d getPose() {
        return currentPose;
    }

    /**
     * updates pose Estimator with vision measurements
     *
     * @param visionPose the pose of the robot from vision
     * @param timeStamp  the time stamp of the vision measurement
     */
    public void addVisionMeasurement(Pose2d visionPose, double timeStamp) {
        poseEstimator.addVisionMeasurement(visionPose, timeStamp);
    }

    /**
     * updates pose Estimator with vision measurements
     *
     * @param visionPose the pose of the robot from vision
     * @param timeStamp  the time stamp of the vision measurement
     * @param stdDevs    the standard deviations of the vision measurements
     */
    public void addVisionMeasurement(Pose2d visionPose, double timeStamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timeStamp, stdDevs);
    }

    /**
     * drives the robot relative to itself
     * @param chassisSpeeds the target chassis speeds of the robot
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        targetStates = driveTrainKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, MAX_FREE_WHEEL_SPEED);

        for (int i = 0; i < 4; i++) {
            targetStates[i] = modules[i].run(targetStates[i]);
        }

        inputs.XspeedInput = chassisSpeeds.vxMetersPerSecond;
        inputs.YspeedInput = chassisSpeeds.vyMetersPerSecond;
        inputs.rotationSpeedInput = chassisSpeeds.omegaRadiansPerSecond;
        Logger.processInputs(getName(), inputs);
    }

    /**
     * drives the robot
     * made for pathplanner driving
     * @param chassisSpeeds the target chassis speeds of the robot
     * @param feedforwards the feedforwards to give the modules
     */
    public void drivePP(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        targetStates = driveTrainKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, MAX_FREE_WHEEL_SPEED);

        for (int i = 0; i < 4; i++) {
            targetStates[i] = modules[i].run(targetStates[i], feedforwards.accelerationsMPSSq()[i]);
        }

        inputs.XspeedInput = chassisSpeeds.vxMetersPerSecond;
        inputs.YspeedInput = chassisSpeeds.vyMetersPerSecond;
        inputs.rotationSpeedInput = chassisSpeeds.omegaRadiansPerSecond;
        Logger.processInputs(getName(), inputs);
    }

    /**
     * drives the robot with the sysID routine
     * not using module optimizations
     * @param states the states to drive the robot in (speeds is voltage)
     *               in order of front left, front right, back left, back right
     */
    public void driveSysID(SwerveModuleState[] states) {
        if(states == null || states.length != 4) return;

        for (int i = 0; i < 4; i++) {
            modules[i].runSysID(states[i].speedMetersPerSecond, states[i].angle);
        }
    }

    public void runSYSIDSteer(Voltage voltage){
        for (int i = 0; i < 4; i++) {
            modules[i].runSysIDSteer(voltage);
        }
    }

    public Command startModuleDriveCalibration() {
        return new InstantCommand(() -> {
            for (int i = 0; i < 4; i++) {
                modules[i].runDriveCalibration();
            }
        }).withName("start module drive calibration").ignoringDisable(true);
    }

    public Command stopModuleDriveCalibration() {
        return new InstantCommand(() -> {
            for (int i = 0; i < 4; i++) {
                modules[i].stopDriveCalibration();
            }
        }).withName("Stop Module Drive Calibration").ignoringDisable(true);
    }

    public Command startModuleSteerCalibration() {
        return new InstantCommand(() -> {
            for (int i = 0; i < 4; i++) {
                modules[i].runSteerCalibration();
            }
        }).withName("start module steer calibration").ignoringDisable(true);
    }

    public Command stopModuleSteerCalibration() {
        return new InstantCommand(() -> {
            for (int i = 0; i < 4; i++) {
                modules[i].stopSteerCalibration();
            }
        }).withName("Stop Module Steer Calibration").ignoringDisable(true);
    }

    /**
     * sets the speed target of the robot during pathplanner driving (overrides the pathplanner)
     * for example used to go towards a note while in-taking
     * @param xSpeed the x speed of the robot in m/s (field relative) (away from the alliance station is positive)
     * @param ySpeed the y speed of the robot in m/s (field relative) (left from the alliance station is positive)
     * @param thetaSpeed the theta speed of the robot in rad/s (left is positive)
     */
    public void overrideXYThetaSpeeds(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier thetaSpeed){
        PPHolonomicDriveController.overrideXYFeedback(xSpeed, ySpeed);
        PPHolonomicDriveController.overrideRotationFeedback(thetaSpeed);
    }

    /**
     * sets the speed target of the robot during pathplanner driving (overrides the pathplanner) in field relative speeds
     * for example used to go towards a note while in-taking
     * @param xSpeed the x speed of the robot in m/s (field relative) (away from the alliance station is positive)
     * @param ySpeed the y speed of the robot in m/s (field relative) (left from the alliance station is positive)
     */
    public void overridePPXSpeeds(DoubleSupplier xSpeed, DoubleSupplier ySpeed){
        PPHolonomicDriveController.overrideXYFeedback(xSpeed, ySpeed);
    }

    /**
     * sets the speed target of the robot during pathplanner driving (overrides the pathplanner) in field relative speeds
     * for example used to go towards a note while in-taking
     * @param xSpeed the x speed of the robot in m/s (field relative) (away from the alliance station is positive)
     */
    public void overridePPXSpeeds(DoubleSupplier xSpeed){
        PPHolonomicDriveController.overrideXFeedback(xSpeed);
    }

    /**
     * sets the speed target of the robot during pathplanner driving (overrides the pathplanner) in field relative speeds
     * for example used to go towards a note while in-taking
     * @param ySpeed the y speed of the robot in m/s (field relative) (left from the alliance station is positive)
     */
    public void overridePPYSpeeds(DoubleSupplier ySpeed) {
        PPHolonomicDriveController.overrideYFeedback(ySpeed);
    }

    /**
     * sets the speed target of the robot during pathplanner driving (overrides the pathplanner)
     * for example used to go towards a note while in-taking
     * @param thetaSpeed the theta speed of the robot in rad/s (left is positive)
     */
    public void overridePPThetaSpeed(DoubleSupplier thetaSpeed){
        PPHolonomicDriveController.overrideRotationFeedback(thetaSpeed);
    }

    /**
     * stops the pathplanner speed overrides (x, y, and theta)
     */
    public void stopPPOverrides(){
        PPHolonomicDriveController.clearFeedbackOverrides();
    }

    /**
     * stops the pathplanner x speed override
     */
    public void stopPPXSpeedOverride(){
        PPHolonomicDriveController.clearXFeedbackOverride();
    }

    /**
     * stops the pathplanner y speed override
     */
    public void stopPPYSpeedOverride(){
        PPHolonomicDriveController.clearYFeedbackOverride();
    }

    /**
     * stops the pathplanner theta speed override
     */
    public void stopPPThetaSpeedOverride(){
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }


    /**
     * path finds a path from the current pose to the target pose
     *
     * @param targetPose  the target pose
     * @param endVelocity the velocity the robot should be in when it reaches the end of the path in m/s
     * @return a command that follows a path to the target pose
     */
    public Command findPath(Pose2d targetPose, double endVelocity) {
        return AutoBuilder.pathfindToPose(targetPose, DriveBaseConstants.PathPlanner.PATH_CONSTRAINTS, endVelocity);
    }

    /**
     * path finds a path from the current pose to the target pose
     *
     * @param targetPose the target pose
     * @return a command that follows a path to the target pose
     */
    public Command findPath(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(targetPose, DriveBaseConstants.PathPlanner.PATH_CONSTRAINTS);
    }

    /**
     * path finds to a given path then follows that path
     *
     * @param targetPath the path to path finds to and follow
     * @return a command that path finds to a given path then follows that path
     */
    public Command pathFindToPathAndFollow(PathPlannerPath targetPath) {
        return AutoBuilder.pathfindThenFollowPath(targetPath, DriveBaseConstants.PathPlanner.PATH_CONSTRAINTS);
    }

    SwerveModulePosition[] previousPositions = new SwerveModulePosition[]{
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()};

    SwerveModulePosition[] positions = new SwerveModulePosition[]{
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()};

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            inputs.currentStates[i] = modules[i].getCurrentState();
            positions[i] = modules[i].modulePeriodic();

            moduleDeltas[i] = new SwerveModulePosition(
                    positions[i].distanceMeters - previousPositions[i].distanceMeters,
                    positions[i].angle
            );

            previousPositions[i] = positions[i].copy();
        }

        gyro.update();
        poseEstimator.update(gyro.getRotation2d(), positions);
        currentPose = poseEstimator.getEstimatedPosition();

        Logger.recordOutput("DriveBase/estimatedPose", currentPose);
        Logger.recordOutput("DriveBase/ChassisSpeeds", getChassisSpeeds());
        Logger.recordOutput("DriveBase/targetStates", targetStates);

        Robot.setRobotPoseField(currentPose);

        inputs.activeCommand = this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None";

        Logger.processInputs(getName(), inputs);
    }
}
