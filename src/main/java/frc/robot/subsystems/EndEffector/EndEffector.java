// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
    private boolean isGpLoaded;

    private double funnelTarget;

    public EndEffector() {
        if (Robot.isSimulation()) {
            io = new EndEffectorIOSim();
        } else {
            io = new EndEffectorIOReal();
        }

        setLoadedValue(Constants.ROBOT_TYPE == Constants.RobotType.COMPETITION);
    }

    public void setRightMotorPower(double PowerToSet) {
        io.setRightMotorPower(PowerToSet);
    }

    public void setLeftMotorPower(double PowerToSet) {
        io.setLeftMotorPower(PowerToSet);
    }

    public boolean isFunnelInClimb(){
        return getFunnelPosition() < EndEffectorConstants.FunnelMotor.CLIMB_POSITION / 2;
    }

    public void moveFunnel(double target) {
        io.moveFunnel(target);
        Logger.recordOutput("EndEffector/funnel target", target);
        funnelTarget = target;
    }

    public void setFunnelVoltage(double voltage) {
        io.setFunnelVoltage(voltage);
    }

    public void stopEndEffectorMotors() {
        io.setLeftMotorPower(0);
        io.setRightMotorPower(0);
    }

    public void stopFunnelMotor() {
        io.stopFunnel();
    }

    public void resetFunnelEncoder() {
        io.resetFunnelEncoder();
    }

    public void setLoadedValue(boolean value) {
        Logger.recordOutput("EndEffector/GP loaded", value);
        SmartDashboard.putBoolean("Is GP Loaded", value);
        isGpLoaded = value;
    }

    public double getFunnelPosition() {
        return inputs.funnelPosition;
    }

    public double getFunnelTarget(){
        return funnelTarget;
    }

    public boolean isGpLoaded() {
        return isGpLoaded;
    }

    public boolean isGpDetected() {
        return inputs.beamBreakValue;
    }

    public void startFunnelPIDCalibration(){
        io.startFunnelPIDCalibration();
    }
    
    public void endFunnelPIDCalibration(){
        io.endFunnelPIDCalibration();
    }

    @Override
    public void periodic() {
        inputs.pose = new Pose3d(EndEffectorConstants.X_ON_ROBOT, EndEffectorConstants.Y_ON_ROBOT, EndEffectorConstants.Z_OFFSET, new Rotation3d(0, -inputs.funnelPosition, 0));
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        String currentCommandName = getCurrentCommand() == null ? "Null" : getCurrentCommand().getName();
        Logger.recordOutput("EndEffector/Current Command", currentCommandName);
    }
}
