package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;


public class vision extends Command {
  private PhotonCamera camera = new PhotonCamera("EndEffectorCamera");
  DriveBase driveBase;
  public vision() 
  { 
    this.driveBase = driveBase;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    var visionResult = camera.getLatestResult();
    double tolerance = 2;
    if(visionResult.hasTargets())
    {
      PhotonTrackedTarget visionTarget = visionResult.getBestTarget();
      if(Math.abs(visionTarget.getYaw()) <= tolerance)
      {
        driveBase.drive(new ChassisSpeeds(1.5,0,0));
      }
    }
    else
    driveBase.drive(new ChassisSpeeds(0,0,0.5));
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.drive(new ChassisSpeeds(0,0,0));
  }

  @Override
  public boolean isFinished() 
  {
    var visionResult = camera.getLatestResult();
    
    if(visionResult.hasTargets())
    {
      PhotonTrackedTarget visionTarget = visionResult.getBestTarget();
    if(visionTarget.getArea() >= 75)
      return true;
    }
    return false;
  }
}
