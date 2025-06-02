package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;
import frc.robot.subsystems.EndEffector.EndEffectorIOReal;

public class Blackbox {
    private static EndEffector endEffector = RobotContainer.endEffector;
    private static BallDropping dropper = RobotContainer.ballDropping;
    private static CommandXboxController controller = RobotContainer.driveController;
    private static DigitalInput beamBreak = ((EndEffectorIOReal)RobotContainer.endEffector.io).beamBreak;

    /**
     * Sets the duty cycle (percent of full power) of the right endeffector motor
     * @param dutyCycle percent of full power to set
     */
    public static InstantCommand endEffectorMotorRightDutyCycle(double dutyCycle){
        return (new InstantCommand(() -> endEffector.setRightMotorPower(dutyCycle)));
    }
    /**
     * Sets the duty cycle (percent of full power) of the left endeffector motor
     * @param dutyCycle percent of full power to set
     */
    public static InstantCommand endEffectorMotorLeftDutyCycle(double dutyCycle){
        return (new InstantCommand(() -> endEffector.setLeftMotorPower(dutyCycle)));
    }
    /**
     * Stops the right endeffector motor
     */
    public static InstantCommand endEffectorMotorLeftStop(){
        return (new InstantCommand(() -> endEffector.setLeftMotorPower(0)));
    }
    /**
     * Stops the left endeffector motor
     */
    public static InstantCommand endEffectorMotorRightStop(){
        return (new InstantCommand(() -> endEffector.setRightMotorPower(0)));
    }

    // TODO: CHECK BALLDROPPER SOFTLIMITS
    /**
     * Move Balldropper to given angle, function ends when at setpoint
     * @param angle Angle (in degrees) to go to
     */
    public static Command ballDropperReachAngle(double angle){
        return (new InstantCommand(() -> dropper.reachAngle(Units.degreesToRotations(angle))))
        .until(() -> (dropper.getAngle() >= Units.degreesToRotations(angle)));
    }
    /**
     * Sets the duty cycle (percent of full power) of the Balldropper motor
     * @param dutyCycle dutyCycle percent of full power to set
     */
    public static InstantCommand ballDropperDutyCycle(double dutyCycle){
        return (new InstantCommand(() -> dropper.setDropperMotorPower(dutyCycle)));
    }
    /**
     * Stops the Balldropper motor
     */
    public static InstantCommand ballDropperStopMotor(){
        return (new InstantCommand(() -> dropper.stopDropperMotor()));
    }

    /**
     * @return BooleanTrigger of controller's circle button
     */
    public static Trigger controllerCircle(){ return controller.b(); }
    /**
     * @return BooleanTrigger of controller's square button
     */
    public static Trigger controllerSquare(){ return controller.x(); }
    /**
     * @return BooleanTrigger of controller's cross button
     */
    public static Trigger controllerCross(){ return controller.a(); }
    /**
     * @return BooleanTrigger of controller's triangle button
     */
    public static Trigger controllerTriangle(){ return controller.y(); }

    // public static Supplier<Double> controllerLeftXAxis() { return controller::getLeftX; }
    // public static Supplier<Double> controllerLeftYAxis() { return controller::getLeftY; }
    // public static Supplier<Double> controllerRightXAxis() { return controller::getRightX; }
    // public static Supplier<Double> controllerRightYAxis() { return controller::getRightY; }

    /**
     * @return BooleanTrigger of the beambreak's state
     */
    public static Trigger beamBreakTrigger() { return new Trigger(() -> beamBreak.get()); }
}
