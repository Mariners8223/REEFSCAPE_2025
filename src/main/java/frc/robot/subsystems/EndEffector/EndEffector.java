// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.EndEffector.EndEffectorIO.EndEffectorInputs;
import frc.robot.Robot;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
    private boolean isGpLoaded;

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

    public void moveFunnel(double target) {
        io.moveFunnel(target);
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
        Logger.recordOutput("EndEffector/gp loaded", value);
        SmartDashboard.putBoolean("Is Gp Loaded", value);
        isGpLoaded = value;
    }

    public double getFunnelPosition() {
        return inputs.funnelPosition;
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
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        String currentCommandName = "None";
        if(getCurrentCommand() != null) {
            currentCommandName = getCurrentCommand().getName();
        }

        Logger.recordOutput("EndEffector/current command", currentCommandName);
        
    }
}
