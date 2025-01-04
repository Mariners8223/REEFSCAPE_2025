package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.util.MarinersController.*;

public class SwerveModuleIODevBot extends SwerveModuleIO {
    private final MarinersController driveMotor;
    private final MarinersController steerMotor;

    private final double DRIVE_KA;


    public SwerveModuleIODevBot(SwerveModule.ModuleName name) {

        DevBotConstants constants = DevBotConstants.values()[name.ordinal()];

        this.DRIVE_KA = constants.DRIVE_KA;

        driveMotor = new MarinersTalonFX(
                name.name() + " Drive Motor",
                MarinersController.ControllerLocation.MOTOR,
                constants.DRIVE_MOTOR_ID,
                constants.DRIVE_MOTOR_PID,
                DevBotConstants.DRIVE_GEAR_RATIO / DevBotConstants.WHEEL_CIRCUMFERENCE_METERS);

        driveMotor.setMotorInverted(constants.DRIVE_INVERTED);

        driveMotor.setCurrentLimits(60, 90);

        driveMotor.setMotorDeadBandVoltage(constants.DRIVE_KS);

        steerMotor = new MarinersSparkBase(
                name.name() + " Steer Motor",
                MarinersController.ControllerLocation.RIO,
                constants.STEER_MOTOR_ID,
                true,
                MarinersSparkBase.MotorType.SPARK_MAX,
                constants.STEER_MOTOR_PID);

        steerMotor.setMotorInverted(constants.STEER_INVERTED);

        steerMotor.setMotorDeadBandVoltage(constants.STEER_KS);

        steerMotor.enablePositionWrapping(-0.5, 0.5);

        steerMotor.setProfile(20, 25);

        CANcoder absEncoder = configCANCoder(constants.ABSOLUTE_ENCODER_ID, constants.ABSOLUTE_ZERO_OFFSET, (int) steerMotor.RUN_HZ);

        steerMotor.setMeasurements(new MarinersMeasurementsCTRE(
                absEncoder.getPosition(),
                absEncoder.getVelocity(),
                1
        ));
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getPosition());
        inputs.currentState.speedMetersPerSecond = driveMotor.getVelocity();

        inputs.drivePositionMeters = driveMotor.getPosition();
    }

    @Override
    public void setDriveMotorReference(double reference) {
        driveMotor.setReference(reference, MarinersController.ControlMode.Velocity);
    }

    @Override
    public void setDriveMotorReference(double reference, double acceleration) {
        driveMotor.setReference(reference, MarinersController.ControlMode.Velocity,
                acceleration * DRIVE_KA);
    }

    @Override
    public void setDriveMotorVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    @Override
    public void setSteerMotorReference(double reference) {
        steerMotor.setReference(reference, MarinersController.ControlMode.ProfiledPosition);
    }

    @Override
    public void setSteerMotorVoltage(double voltage) {
        steerMotor.setVoltage(voltage);
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        driveMotor.setMotorIdleMode(isBrakeMode);
        steerMotor.setMotorIdleMode(isBrakeMode);
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


}
