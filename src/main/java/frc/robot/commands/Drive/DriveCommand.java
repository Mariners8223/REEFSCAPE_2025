package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.DriveBase;

import frc.robot.subsystems.DriveTrain.SwerveModules.DevBotConstants;

import static frc.robot.subsystems.DriveTrain.DriveBaseConstants.DISTANCE_BETWEEN_WHEELS;

public class DriveCommand extends Command {

    private final DriveBase driveBase;
    private final CommandXboxController controller;

    private static double MAX_FREE_WHEEL_SPEED;
    private static double MAX_OMEGA_RAD_PER_SEC;

    public DriveCommand(DriveBase driveBase, CommandXboxController controller) {
        this.driveBase = driveBase;
        this.controller = controller;
        addRequirements(this.driveBase);
        setName("DriveCommand");

        MAX_FREE_WHEEL_SPEED = DevBotConstants.MAX_WHEEL_LINEAR_VELOCITY;

        double driveBaseRadius = Math.hypot(DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2);

        MAX_OMEGA_RAD_PER_SEC = MAX_FREE_WHEEL_SPEED / driveBaseRadius;
    }

    @Override
    public void initialize() {
        driveBase.drive(new ChassisSpeeds());
    }

    public static double deadBand(double value) {
        return Math.abs(value) > 0.1 ? value : 0;
    }

    public static void halfSpeed(){
        MAX_FREE_WHEEL_SPEED = DevBotConstants.MAX_WHEEL_LINEAR_VELOCITY / 2;

        double driveBaseRadius = Math.hypot(DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2);

        MAX_OMEGA_RAD_PER_SEC = MAX_FREE_WHEEL_SPEED / driveBaseRadius;        
    }

    public static void normalSpeed(){
        MAX_FREE_WHEEL_SPEED = DevBotConstants.MAX_WHEEL_LINEAR_VELOCITY;

        double driveBaseRadius = Math.hypot(DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2);

        MAX_OMEGA_RAD_PER_SEC = MAX_FREE_WHEEL_SPEED / driveBaseRadius;  
    }


    @Override
    public void execute() {
        //calculates a value from 1 to the max wheel speed based on the R2 axis
        // double R2Axis = (1 - (0.5 + controller.getR2Axis() / 2)) * (driveBase.MAX_FREE_WHEEL_SPEED - 1) + 1;
        // double R2Axis  = 1 - (0.5 + controller.getRightTriggerAxis() / 2);
        double R2Axis  = 1 - controller.getRightTriggerAxis();

        if(R2Axis <= 0.1) {
            R2Axis = 0.1;
        }

        //sets the value of the 3 vectors we need (accounting for drift)
        double leftX = -deadBand(controller.getLeftY());
        double leftY = -deadBand(controller.getLeftX());
        double rightX = -deadBand(controller.getRightX());

        leftX *= R2Axis * MAX_FREE_WHEEL_SPEED;
        leftY *= R2Axis * MAX_FREE_WHEEL_SPEED;
        rightX *= R2Axis * MAX_OMEGA_RAD_PER_SEC;

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(leftX, leftY, rightX);

        Rotation2d gyroAngle = driveBase.getRotation2d();

        if(Robot.isRedAlliance) gyroAngle = gyroAngle.plus(Rotation2d.fromDegrees(180));

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, gyroAngle);

        //drives the robot with the values
        driveBase.drive(robotRelativeSpeeds);
    }
}
