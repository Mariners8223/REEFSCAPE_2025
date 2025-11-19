

import java.security.PrivateKey;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;;


public class wooooooooow extends Command {
    private PhotonCamera camera = new PhotonCamera("EndEffectorCamera");
    private final DriveBase driveBase;
    double tolerance = 10;
    public wooooooooow(DriveBase driveBase)
    {
        this.driveBase = driveBase;
        addRequirements(this.driveBase);
    }
    
    @Override
    public void initialize() {

    }   

    @Override
    public void execute() {
        var visionResult = camera.getLatestResult();

        PhotonTrackedTarget visiontTarget = visionResult.getBestTarget();

        if(visionResult.hasTargets())
        {
            if(visiontTarget.getFiducialId() == 7) 
            {
                if (Math.abs(visiontTarget.getYaw()) <= tolerance)
                {

                }

                driveBase.drive(new ChassisSpeeds(1,0, 12));
                if(visiontTarget.getArea() == 75)
                {
                    //
                    //hello world
                }
            }
        }



    }

    public void end() {

    }

    public boolean isFinished() {

        return false;
    }
}
