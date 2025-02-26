package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSimMotor;
import frc.util.PIDFGains;

public class SwerveModuleIOSIM extends SwerveModuleIO {
    private final MarinersController driveMotor;
    private final MarinersController steerMotor;

    private final double DRIVE_KA;

    public SwerveModuleIOSIM(SwerveModule.ModuleName name) {
        DCMotor driveMotorModel;
        DCMotor steerMotorModel;

        PIDFGains driveMotorPIDController, steerMotorPIDController;

        double DRIVE_KS, DRIVE_KV, STEER_KV, STEER_KA, DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEEL_RADIUS_METERS, WHEEL_CIRCUMFERENCE_METERS;


        DRIVE_GEAR_RATIO = DevBotConstants.DRIVE_GEAR_RATIO;
        STEER_GEAR_RATIO = DevBotConstants.STEER_GEAR_RATIO;
        WHEEL_CIRCUMFERENCE_METERS = DevBotConstants.WHEEL_CIRCUMFERENCE_METERS;
        WHEEL_RADIUS_METERS = DevBotConstants.WHEEL_RADIUS_METERS;

        DevBotConstants constants = DevBotConstants.values()[name.ordinal()];

        driveMotorPIDController = constants.DRIVE_MOTOR_PID;
        steerMotorPIDController = constants.STEER_MOTOR_PID;

        driveMotorModel = DCMotor.getKrakenX60(1);
        steerMotorModel = DCMotor.getNEO(1);

        DRIVE_KS = constants.DRIVE_KS;
        DRIVE_KV = constants.DRIVE_KV;
        DRIVE_KA = constants.DRIVE_KA;

        STEER_KV = constants.STEER_KV;
        STEER_KA = constants.STEER_KA;

        driveMotor = new MarinersSimMotor(name.name() + " Drive Motor", driveMotorModel,
                DRIVE_KV * WHEEL_RADIUS_METERS, DRIVE_KA * WHEEL_RADIUS_METERS,
                DRIVE_GEAR_RATIO, 1 / WHEEL_CIRCUMFERENCE_METERS);

        driveMotor.setPIDF(driveMotorPIDController);

        driveMotor.setStaticFeedForward(DRIVE_KS);

        driveMotor.setFeedForward(DRIVE_KV);

        steerMotor = new MarinersSimMotor(name.name() + " Steer Motor", steerMotorModel,
                STEER_KV / (Math.PI * 2), STEER_KA / (Math.PI * 2), STEER_GEAR_RATIO, 1);

        steerMotor.setPIDF(steerMotorPIDController);

        steerMotor.enablePositionWrapping(-0.5, 0.5);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentState.speedMetersPerSecond = driveMotor.getVelocity();

        inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getPosition());

        inputs.drivePositionMeters = driveMotor.getPosition();
    }

    @Override
    public void setDriveMotorReference(double reference) {
        driveMotor.setReference(reference, MarinersController.ControlMode.Velocity);
    }

    @Override
    public void setDriveMotorReference(double reference, double accelerationFeedForward) {
        driveMotor.setReference(reference, MarinersController.ControlMode.Velocity, accelerationFeedForward * DRIVE_KA);
    }

    @Override
    public void setDriveMotorVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    @Override
    public void setSteerMotorReference(double reference) {
        steerMotor.setReference(reference, MarinersController.ControlMode.Position);
    }

    @Override
    public void setSteerMotorVoltage(double voltage) {
        steerMotor.setVoltage(voltage);
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        DriverStation.reportWarning("dummy this is a simulation", false);
    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.resetMotorEncoder();
    }

    @Override
    void startDriveCalibration() {
        driveMotor.startPIDTuning();
    }

    @Override
    void endDriveCalibration() {
        driveMotor.startPIDTuning();
    }

    @Override
    void startSteerCalibration() {
        steerMotor.startPIDTuning();
    }

    @Override
    void endSteerCalibration() {
        steerMotor.startPIDTuning();
    }


    /**
     * A fake class for replaying data
     */
    public static class SwerveModuleIOReplay extends SwerveModuleIO {
        @Override
        public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        }

        @Override
        public void setDriveMotorReference(double reference) {

        }

        @Override
        public void setDriveMotorReference(double reference, double accelerationFeedForward) {

        }

        @Override
        public void setDriveMotorVoltage(double voltage) {

        }

        @Override
        public void setSteerMotorReference(double reference) {

        }

        @Override
        public void setSteerMotorVoltage(double voltage) {

        }

        @Override
        public void setIdleMode(boolean isBrakeMode) {
        }

        @Override
        public void resetDriveEncoder() {
        }

        @Override
        void startDriveCalibration() {

        }

        @Override
        void endDriveCalibration() {

        }

        @Override
        void startSteerCalibration() {

        }

        @Override
        void endSteerCalibration() {

        }
    }
}
