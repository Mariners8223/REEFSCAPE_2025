// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersTalonFX;

public class EndEffectorIOReal implements EndEffectorIO {
    private final VictorSPX RightMotor;
    private final VictorSPX LeftMotor;
    private final MarinersTalonFX FunnelMotor;
    private final DigitalInput beamBreak;


    public EndEffectorIOReal() {
        RightMotor = configureEndEffectorMotor(EndEffectorConstants.RIGHT_ID, EndEffectorConstants.RIGHT_INVERTED);
        LeftMotor = configureEndEffectorMotor(EndEffectorConstants.LEFT_ID, EndEffectorConstants.LEFT_INVERTED);
        FunnelMotor = configureFunnelMotor();
        beamBreak = new DigitalInput(EndEffectorConstants.BEAM_BREAK_PORT);

        resetFunnelEncoder();
    }

    private VictorSPX configureEndEffectorMotor(int ID, boolean isInverted) {
        VictorSPX motor = new VictorSPX(ID);
        motor.setInverted(isInverted);
        motor.setNeutralMode(NeutralMode.Brake);
        return motor;
    }

    private MarinersTalonFX configureFunnelMotor() {
        MarinersTalonFX motor = new MarinersTalonFX("Funnel Motor", EndEffectorConstants.FunnelMotor.CONTROLLER_LOCATION,
                EndEffectorConstants.FunnelMotor.MOTOR_ID, EndEffectorConstants.FunnelMotor.PID_GAINS, EndEffectorConstants.FunnelMotor.GEAR_RATIO);
        motor.setMotorInverted(EndEffectorConstants.FunnelMotor.IS_INVERTED);
        motor.setMotorIdleMode(true);
        motor.enableSoftLimits(EndEffectorConstants.FunnelMotor.CLIMB_POSITION - 0.05, EndEffectorConstants.FunnelMotor.COLLECT_POSITION + 0.05);

        motor.setMaxMinOutput(8, 7);
        return motor;
    }

    public void setRightMotorPower(double PowerToSet) {
        double realPower = MathUtil.clamp(PowerToSet, -EndEffectorConstants.MAX_MOTOR_POWER,
                EndEffectorConstants.MAX_MOTOR_POWER);

        RightMotor.set(VictorSPXControlMode.PercentOutput, realPower);
    }

    public void setLeftMotorPower(double PowerToSet) {
        double realPower = MathUtil.clamp(PowerToSet, -EndEffectorConstants.MAX_MOTOR_POWER,
                EndEffectorConstants.MAX_MOTOR_POWER);

        LeftMotor.set(VictorSPXControlMode.PercentOutput, realPower);
    }

    public void resetFunnelEncoder() {
        FunnelMotor.resetMotorEncoder();
    }

    public void moveFunnel(double target) {
        FunnelMotor.setReference(target, MarinersSparkBase.ControlMode.Position);
    }

    public void setFunnelVoltage(double voltage) {
        FunnelMotor.setVoltage(voltage);
    }

    public void stopFunnel() {
        FunnelMotor.stopMotor();
    }

    public void startFunnelPIDCalibration(){
        FunnelMotor.startPIDTuning();
    }

    public void endFunnelPIDCalibration(){
        FunnelMotor.stopPIDTuning();
    }

    public void Update(EndEffectorInputs inputs) {
        inputs.rightPower = RightMotor.getMotorOutputPercent();
        inputs.leftPower = LeftMotor.getMotorOutputPercent();
        inputs.funnelPosition = FunnelMotor.getPosition();
        inputs.beamBreakValue = EndEffectorConstants.BEAM_BREAK_INVERTED != beamBreak.get();
    }
}
