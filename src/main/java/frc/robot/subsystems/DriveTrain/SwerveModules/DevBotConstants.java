package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.util.PIDFGains;

public enum DevBotConstants {
    FRONT_LEFT(6, 10, 14,
            false, false, -0.361328125,
            new PIDFGains(50, 0, 0), //Drive motor PID
            new PIDFGains(25, 20, 0, 0, 0.001, 0.05), //Steer motor PID
            0.10914, 2.7, 0.23431, 0.35464, 1.7245, 0.23146),

    FRONT_RIGHT(12, 4, 5,
    false, false, 0.09130859375,
            new PIDFGains(50, 0, 0), //Drive motor PID
            new PIDFGains(25, 20, 0, 0, 0.001, 0.05), //Steer motor PID
            0.1176, 2.7, 0.22767, 0.28713, 1.6793, 0.17791),

    BACK_LEFT(9, 7, 11,
    false, false, -0.00244140625,
            new PIDFGains(50, 0, 0), //Drive motor PID
            new PIDFGains(25, 20, 0, 0, 0.001, 0.05), //Steer motor PID
            0.12387, 2.7, 0.33572, 0.22659, 1.7941, 0.39017),

    BACK_RIGHT(3, 13, 8,
    false, false, -0.3974609375,
            new PIDFGains(50, 0, 0), //Drive motor PID
            new PIDFGains(25, 20, 0, 0, 0.001, 0.05), //Steer motor PID
            0.096306, 2.7, 0.30283, 0.43868, 1.8036, 0.35004);

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double WHEEL_RADIUS_METERS = 0.0508;
    public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double MAX_WHEEL_LINEAR_VELOCITY = 4;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 112;
    public static final int DRIVE_MOTOR_CURRENT_THRESHOLD = 70;

    //acceleration and jerk constraints for the drive motor
    public static final TrapezoidProfile.Constraints DRIVE_CONSTRAINTS = new TrapezoidProfile.Constraints(18, 100);

    //velocity and acceleration constraints for the steer motor
    public static final TrapezoidProfile.Constraints STEER_CONSTRAINTS = new TrapezoidProfile.Constraints(20, 25);

    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
        WHEEL_RADIUS_METERS,
        MAX_WHEEL_LINEAR_VELOCITY,
        0.9,
        DCMotor.getKrakenX60(1),
        DRIVE_GEAR_RATIO,
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