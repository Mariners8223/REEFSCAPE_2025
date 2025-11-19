// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControllerLocation;

/** Add your docs here. */
public class ClimbIOFalcon implements ClimbIO{
    MarinersTalonFX motor;

    public ClimbIOFalcon(){
        motor = configureMotor();
    }

    public MarinersTalonFX configureMotor(){
        MarinersTalonFX motor = new MarinersTalonFX(
            "Climb Motor",
            ControllerLocation.MOTOR,
            ClimbConstants.MOTOR_ID,
            ClimbConstants.GEAR_RATIO * ClimbConstants.ROTATIONS_TO_METERS);
        
        // motor.enableSoftLimits(ClimbConstants.SOFT_MINIMUM, ClimbConstants.SOFT_MAXIMUM);
        motor.setMotorIdleMode(true);
        motor.setMotorInverted(ClimbConstants.IS_INVERTED);
        motor.setMaxMinOutput(0, -1 * 12);//-0.4
        motor.enableSoftLimits(ClimbConstants.SOFT_MINIMUM, ClimbConstants.START_POSITION);

        return motor;
    }

    public void setPower(double power) { motor.setDutyCycle(power); }

    public void resetPosition() { motor.setMotorEncoderPosition(ClimbConstants.START_POSITION); }
    public double getPosition() { return motor.getPosition(); }

    public void Update(ClimbInputs inputs){
        inputs.height = getPosition();
        inputs.pose = new Pose3d(0, 0, 0.2 + inputs.height, new Rotation3d());
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        motor.setMotorIdleMode(isBrake);
    }
}
