// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionProMax;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;

public class vision extends Command {
  private PhotonCamera camera = new PhotonCamera("EndEffectorCamera");
  DriveBase driveBase;
  public vision() {

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    var visionResult = camera.getLatestResult();
    double tolerance=2;
    if(visionResult.hasTargets()){
      PhotonTrackedTarget visionTarget = visionResult.getBestTarget();
      if(Math.abs(visionTarget.getYaw())<=tolerance){
        driveBase.drive(new ChassisSpeeds(0, 0, visionTarget.getYaw()));
      }
      driveBase.drive(new ChassisSpeeds(2, 0, 0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.drive(new ChassisSpeeds(0, 0, 0));;

  }

  @Override
  public boolean isFinished() {
    var visionResult = camera.getLatestResult();
    visionResult.hasTargets();
    PhotonTrackedTarget visionTarget = visionResult.getBestTarget();
    if(visionTarget.getArea()>75) return true;
    return false;
  }
}
