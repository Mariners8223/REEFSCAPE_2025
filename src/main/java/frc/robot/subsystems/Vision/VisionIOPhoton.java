package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionIOPhoton implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private final Supplier<Pose2d> referncePoseSupplier;


    public VisionIOPhoton(CameraConstants cameraConstants, AprilTagFieldLayout fieldLayout, Supplier<Pose2d> referencePoseSupplier) {
        camera = new PhotonCamera(cameraConstants.cameraName);

        poseEstimator = new PhotonPoseEstimator(fieldLayout, VisionConstants.MAIN_STRATEGY, cameraConstants.robotToCamera);

        poseEstimator.setMultiTagFallbackStrategy(VisionConstants.FALLBACK_STRATEGY);

        this.referncePoseSupplier = referencePoseSupplier;
    }


    @Override
    public void update(VisionInputsAutoLogged inputs) {
        inputs.isConnected = camera.isConnected();

        if(!inputs.isConnected){
            inputs.visionFrames = emptyFrame;
            return;
        }

        var results = camera.getAllUnreadResults();

        inputs.visionFrames = new VisionFrame[results.size()];

        for(int i = 0; i < results.size(); i++){
            inputs.visionFrames[i] = generateFrame(results.get(i));
            ArrayList<Integer> targetIDs = new ArrayList<>();

            for(PhotonTrackedTarget target : results.get(i).getTargets()){
                targetIDs.add(target.getFiducialId());
            }

            inputs.targetIDs = targetIDs.stream().mapToInt(Integer::intValue).toArray();
        }

    }

    /**
     * generates a VisionFrame from a PhotonPipelineResult
     * @param result the result to generate the frame from
     * @return the generated frame
     */
    private VisionFrame generateFrame(PhotonPipelineResult result){
        if(!result.hasTargets()){
            return emptyFrame[0];
        }

        poseEstimator.setReferencePose(referncePoseSupplier.get());

        Optional<EstimatedRobotPose> poseEstimatorResult = poseEstimator.update(result);

        if(poseEstimatorResult.isEmpty()){
            return emptyFrame[0];
        }

        EstimatedRobotPose robotPose = poseEstimatorResult.get();

        EstimationType estimationType;
        double poseAmbiguity;
        double averageTargetDist = 0;
        if(result.multitagResult.isPresent()){
            estimationType = EstimationType.MULTIPLE_TARGETS;
            poseAmbiguity = result.multitagResult.get().estimatedPose.ambiguity;

            for(PhotonTrackedTarget target : result.getTargets()){
                double distanceToTag = target.getBestCameraToTarget().getTranslation().getNorm();
                averageTargetDist += distanceToTag;
            }
        }
        else{
            estimationType = EstimationType.SINGLE_TARGET;
            poseAmbiguity = result.getTargets().get(0).poseAmbiguity;

            averageTargetDist = result.getTargets().get(0).getBestCameraToTarget().getTranslation().getNorm();
        }

        return new VisionFrame(
                true,
                result.getTimestampSeconds(),
                RobotController.getMeasureTime().in(Units.Seconds) - result.getTimestampSeconds(),
                robotPose.estimatedPose,
                poseAmbiguity,
                estimationType,
                averageTargetDist / result.getTargets().size(),
                result.getTargets().size()
        );
    }
}
