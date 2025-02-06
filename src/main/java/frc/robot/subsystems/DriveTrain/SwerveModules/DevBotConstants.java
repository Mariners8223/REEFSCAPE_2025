package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.util.PIDFGains;

public enum DevBotConstants {
    FRONT_LEFT(6, 10, 14,
            false, false, -0.361328125,
            new PIDFGains(3, 0, 0, 1.8749), //Drive mtor PID
            new PIDFGains(35, 25, 0, 0, 0.001, 0.01), //Steer motor PID
            0.13777, 1.8887, 0.18407, 0.19704, 1.7025, 0.10185),

    FRONT_RIGHT(12, 4, 5,
    false, false, 0.09130859375,
            new PIDFGains(3, 0, 0, 1.8552), //Drive motor PID
            new PIDFGains(35, 25, 0, 0, 0.001, 0.01), //Steer motor PID
            0.063408, 1.8552, 0.10686, 0.19704, 1.7025, 0.10185),

    BACK_LEFT(9, 7, 11,
    false, false, -0.00244140625,
            new PIDFGains(3, 0, 0, 1.9608), //Drive motor PID
            new PIDFGains(35, 25, 0, 0, 0.001, 0.01), //Steer motor PID
            0.19252, 1.9943, 0.14615, 0.2, 1.7386, 0.12048  ),

    BACK_RIGHT(3, 13, 8,
    false, false, -0.3974609375,
            new PIDFGains(3, 0, 0, 1.8443), //Drive motor PID
            new PIDFGains(35, 25, 0, 0, 0.001, 0.01), //Steer motor PID
            0.081338, 1.8486, 0.087161, 0.14, 1.602, 0.11135);

    public static final double DRIVE_GEAR_RATIO = 5.14;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double WHEEL_RADIUS_METERS = 0.0508;
    public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double MAX_WHEEL_LINEAR_VELOCITY = 4.5;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50;
    public static final int DRIVE_MOTOR_CURRENT_THERSHOLD = 80;

    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
        WHEEL_RADIUS_METERS,
        MAX_WHEEL_LINEAR_VELOCITY,
        0.9,
        DCMotor.getKrakenX60(1),
        DRIVE_MOTOR_CURRENT_LIMIT,
        1);

    /**
     * the motor id for the drive motor
     */
    public final int DRIVE_MOTOR_ID;

    /**
     * the motor id for the steer motor
     */
    public final int STEER_MOTOR_ID;

    /**
     * the motor id for the absolute encoder
     */
    public final int ABSOLUTE_ENCODER_ID;

    /**
     * if the drive motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean DRIVE_INVERTED;

    /**
     * if the steer motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean STEER_INVERTED;

    /**
     * the offset between the zero of the magnet of the encoder and the zero of the module (in rotations)
     */
    public final double ABSOLUTE_ZERO_OFFSET;

    /**
     * the PIDF gains for the drive motor
     */
    public final PIDFGains DRIVE_MOTOR_PID;

    /**
     * the PIDF gains for the steer motor
     */
    public final PIDFGains STEER_MOTOR_PID;

    /**
     * the kS for the drive motor
     * the voltage needed to overcome static friction
     * units are volts
     */
    public final double DRIVE_KS;

    /**
     * the kV for the drive motor
     * units are volts per radian per second (V/rad/s)
     */
    public final double DRIVE_KV;

    /**
     * the kA for the drive motor
     * units are volts per radian per second squared (V/rad/s^2)
     */
    public final double DRIVE_KA;

    /**
     * the kS for the steer motor
     * the voltage needed to overcome static friction
     * units are volts
     */
    public final double STEER_KS;

    /**
     * the kV for the steer motor
     * units are volts per radian per second (V/rad/s)
     */
    public final double STEER_KV;

    /**
     * the kA for the steer motor
     * units are volts per radian per second squared (V/rad/s^2)
     */
    public final double STEER_KA;

    DevBotConstants(int DRIVE_MOTOR_ID, int STEER_MOTOR_ID, int ABSOLUTE_ENCODER_ID, boolean driveInverted,
                     boolean steerInverted, double absZeroOffset, PIDFGains driveMotorPID, PIDFGains steerMotorPID,
                     double DRIVE_KS, double DRIVE_KV, double DRIVE_KA, double STEER_KS, double STEER_KV, double STEER_KA) {
        this.DRIVE_INVERTED = driveInverted;
        this.STEER_INVERTED = steerInverted;
        this.ABSOLUTE_ZERO_OFFSET = absZeroOffset;
        this.DRIVE_MOTOR_PID = driveMotorPID;
        this.STEER_MOTOR_PID = steerMotorPID;

        this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
        this.STEER_MOTOR_ID = STEER_MOTOR_ID;
        this.ABSOLUTE_ENCODER_ID = ABSOLUTE_ENCODER_ID;

        this.DRIVE_KS = DRIVE_KS;
        this.DRIVE_KV = DRIVE_KV;
        this.DRIVE_KA = DRIVE_KA;

        this.STEER_KS = STEER_KS;
        this.STEER_KV = STEER_KV;
        this.STEER_KA = STEER_KA;
    }
}
