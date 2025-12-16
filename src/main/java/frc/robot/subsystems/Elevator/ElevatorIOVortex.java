// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersController.ControlMode;

/** Add your docs here. */
public class ElevatorIOVortex implements ElevatorIO {
    private final MarinersSparkBase motorLead;
    @SuppressWarnings("unused") 
    private final MarinersSparkBase motorFollow;

    public ElevatorIOVortex(){
        this.motorLead = configureLeadMotor();
        this.motorFollow = configureFollowMotor();
    }

    private MarinersSparkBase configureLeadMotor(){
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Lead Elevator Motor", ElevatorConstants.LeadMotor.CONTROLLER_LOCATION, 
            ElevatorConstants.LeadMotor.MOTOR_ID, ElevatorConstants.LeadMotor.IS_BRUSHLESS, 
            ElevatorConstants.LeadMotor.MOTOR_TYPE, ElevatorConstants.PID_GAINS,
            ElevatorConstants.GEAR_RATIO / ElevatorConstants.PULLEY_EXTENSION_RATIO);

        motor.setStaticFeedForward(ElevatorConstants.STATIC_FEEDFORWARD);

        motor.enableSoftLimits(ElevatorConstants.SOFT_MINIMUM, ElevatorConstants.SOFT_MAXIMUM);

        motor.setMaxMinOutput(8, 8);

        motor.setProfile(ElevatorConstants.PROFILE);

        motor.setMotorInverted(ElevatorConstants.LeadMotor.IS_INVERTED);
        motor.setMotorIdleMode(true);

        // motor.setCurrentLimits(60, 70);

        return motor;
    }

    private MarinersSparkBase configureFollowMotor(){
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Follow Elevator Motor", ElevatorConstants.FollowMotor.CONTROLLER_LOCATION, 
            ElevatorConstants.FollowMotor.MOTOR_ID, ElevatorConstants.FollowMotor.IS_BRUSHLESS, ElevatorConstants.FollowMotor.MOTOR_TYPE);
        
        motor.setMotorAsFollower(this.motorLead, ElevatorConstants.FollowMotor.IS_INVERTED);

        motor.setMotorIdleMode(true);

        return motor;
    }

    public void resetMotorEncoder(){
        motorLead.setMotorEncoderPosition(ElevatorLevel.Bottom.getHeight());
    }

    public void moveMotorByPosition(double position){
        motorLead.setReference(position, ControlMode.ProfiledPosition, ElevatorConstants.FEED_FORWARD);
    }

    public void setVoltage(double voltage){
        motorLead.setVoltage(voltage);
    }

    public void Update(ElevatorInputs inputs){
        inputs.elevatorHeight = motorLead.getPosition();
        inputs.elevator3DPose = new Pose3d(ElevatorConstants.X_ON_ROBOT, ElevatorConstants.Y_ON_ROBOT, ElevatorConstants.Z_OFFSET + inputs.elevatorHeight, new Rotation3d());
    }
}
