// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;


import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersSparkBase.MotorType;


public class BallDroppingIOReal implements BallDroppingIO{
    private final MarinersController angleMotor;
    private final VictorSPX dropperMotor;

    public BallDroppingIOReal(){
       angleMotor = configureAngleMotor();
       dropperMotor = configureDropperMotor();
    }

    private MarinersController configureAngleMotor(){
        MarinersController motor = new MarinersSparkBase("angle motor", BallDroppingConstants.AngleMotor.CONTROLLER_LOCATION,
        BallDroppingConstants.AngleMotor.ID, true, MotorType.SPARK_FLEX);

        motor.setPIDF(BallDroppingConstants.AngleMotor.ANGLE_PID);
        motor.getMeasurements().setGearRatio(BallDroppingConstants.AngleMotor.GEAR_RATIO);

        motor.setMotorInverted(BallDroppingConstants.AngleMotor.IS_INVERTED);
        
        motor.setMaxMinOutput(3, 3);

        motor.setMotorIdleMode(true);

        motor.enableSoftLimits(BallDroppingConstants.AngleMotor.ANGLE_TO_RESET, BallDroppingConstants.AngleMotor.ANGLE_TO_REACH_TOP + 0.1); 
        // TODO: CHECK ACTUAL SOFT LIMITS
        
        return motor;
    }
    private VictorSPX configureDropperMotor(){
        VictorSPX motor = new VictorSPX(BallDroppingConstants.DropperMotor.ID);
        motor.setInverted(BallDroppingConstants.DropperMotor.IS_INVERTED);
        return motor;
    }


    //angle motor implement
    public void resetAngleEncoder(){
        angleMotor.resetMotorEncoder();
    }

    public void reachAngle(double angleToReach){
        angleMotor.setReference(angleToReach, ControlMode.Position, calculateFeedForward(angleMotor.getPosition()));
    }

    /**
     * calculate the feed forward for the motor based on the current position (angle) of the motor
     * @param motorRotation the current position of the motor
     * @return the feed forward value
     */
    private double calculateFeedForward(double motorRotation){
        return Math.sin(motorRotation * 2 * Math.PI) * BallDroppingConstants.AngleMotor.MOTOR_FEED_FORWARD;
    }

    //dropping motor implement
    public void setDropperMotorPower(double dropperPower){
        double realPower = MathUtil.clamp(dropperPower,-BallDroppingConstants.DropperMotor.MAX_DROPPER_POWER,
        BallDroppingConstants.DropperMotor.MAX_DROPPER_POWER);

        dropperMotor.set(VictorSPXControlMode.PercentOutput, realPower);
    }

    public void Update(BallDroppingInputs inputs){
        inputs.angle = angleMotor.getPosition();
        inputs.dropperPower = dropperMotor.getMotorOutputPercent();
        inputs.pose = new Pose3d(BallDroppingConstants.X_ON_ROBOT, BallDroppingConstants.Y_ON_ROBOT, BallDroppingConstants.Z_OFFSET, new Rotation3d(0, inputs.angle, 0));
    }

    @Override
    public void setVoltage(double voltage) {
       angleMotor.setVoltage(voltage);
    }


}
