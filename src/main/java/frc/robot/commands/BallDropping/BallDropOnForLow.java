// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallDropping;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.BallDropping.BallDroppingConstants;
import frc.robot.subsystems.BallDropping.BallDroppingConstants.AngleMotor;
import frc.robot.subsystems.BallDropping.BallDroppingConstants.DropperMotor;

public class  BallDropOnForLow extends Command {
  private final BallDropping ballDrop;
  
  public BallDropOnForLow(BallDropping ballDrop) {
    this.ballDrop = ballDrop;
    addRequirements(ballDrop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballDrop.setDropperMotorPower(DropperMotor.POWER_TO_REACH);
    ballDrop.reachAngle(AngleMotor.ANGLE_TO_REACH_BOTTOM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(ballDrop.getAngle() - AngleMotor.ANGLE_TO_REACH_BOTTOM) <= BallDroppingConstants.AngleMotor.ANGLE_TOLERANCE;
  }
}
