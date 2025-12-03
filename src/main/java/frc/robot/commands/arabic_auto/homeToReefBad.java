// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arabic_auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class homeToReefBad extends Command {
  AutoBuilder AutoBuilder;
  PathPlannerPath path;
  DriveBase DriveBase;
  int timer;
  // Creates a new homeToReefBad. 
  public homeToReefBad(DriveBase driveBase) throws FileVersionException, IOException, ParseException {
    DriveBase = driveBase;
    AutoBuilder = new AutoBuilder();
    path = PathPlannerPath.fromPathFile("path to reef 1.path");
    
    addRequirements(DriveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    com.pathplanner.lib.auto.AutoBuilder.pathfindThenFollowPath(path, null);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer > 10000) return true;
    return false;
  }
}
