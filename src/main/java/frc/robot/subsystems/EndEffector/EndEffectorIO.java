// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface EndEffectorIO {
    @AutoLog
    class EndEffectorInputs{
        double rightPower;
        double leftPower;
        double funnelPosition;
        boolean beamBreakValue;
        Pose3d pose;
    }

    void setRightMotorPower(double powerToSet);

    void setLeftMotorPower(double powerToSet);

    void resetFunnelEncoder();

    void moveFunnel(double target);

    void setFunnelVoltage(double voltage);

    void stopFunnel();

    void startFunnelPIDCalibration();

    void endFunnelPIDCalibration();
    
    void Update(EndEffectorInputs inputs);
}
