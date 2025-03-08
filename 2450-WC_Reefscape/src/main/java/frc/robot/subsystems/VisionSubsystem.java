package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera frontCamera = new PhotonCamera("frontCamera");
    //PhotonCamera backCamera = new PhotonCamera("backCamera");

    List<PhotonPipelineResult> frontCameraResults;
    List<PhotonPipelineResult> backCameraResults;

    PhotonTrackedTarget frontCameraBestTarget;
    PhotonTrackedTarget backCameraBestTarget;

    PhotonPipelineResult frontResult;
    PhotonPipelineResult backResult;

    boolean frontCameraHasTarget;
    boolean backCameraHasTarget;

    double apriltagX = 0.0;
    double apriltagY = 0.0;
    double apriltagZ = 0.0;

    ShuffleboardTab tab;

    // AprilTagFieldLayout fieldLayout = AprilTagFieldLayout(Path);

    PhotonPoseEstimator frontPoseEstimator = new PhotonPoseEstimator(null, 
                                                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                                                        new Transform3d(VisionConstants.frontCameraForwardOffset, 
                                                                        VisionConstants.frontCameraRightOffset, 
                                                                        VisionConstants.frontCameraUpOffest,
                                                                        new Rotation3d(0, 0, 0)));
    EstimatedRobotPose frontPoseEstimate;

    /*
    PhotonPoseEstimator backPoseEstimator = new PhotonPoseEstimator(null, 
                                                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                                                        new Transform3d(VisionConstants.backCameraForwardOffset, 
                                                                        VisionConstants.backCameraLeftOffest, 
                                                                        VisionConstants.backCameraUpOffest,
                                                                        new Rotation3d(0, 0, Math.PI)));
    */

    public VisionSubsystem() {
        
    }

    @Override
    public void periodic() {
        // Read in relevant data from the Camera
        frontCameraResults = frontCamera.getAllUnreadResults();
       // backCameraResults = backCamera.getAllUnreadResults();

        if (!frontCameraResults.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            frontResult = frontCameraResults.get(frontCameraResults.size() - 1);
            if (frontResult.hasTargets()) {
                // At least one AprilTag was seen by the camera
                frontCameraBestTarget = frontResult.getBestTarget();
            }
            frontCameraHasTarget = frontResult.hasTargets();
        }

        /*
        if (!backCameraResults.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            backResult = backCameraResults.get(backCameraResults.size() - 1);
            if (backResult.hasTargets()) {
                // At least one AprilTag was seen by the camera
                backCameraBestTarget = frontResult.getBestTarget();
                    // }
                // }
            }
            frontCameraHasTarget = frontResult.hasTargets();
        }
        */

        Optional<EstimatedRobotPose> frontPoseEstimateOptional = frontPoseEstimator.update(frontResult);
        if (frontPoseEstimateOptional.isPresent()) {
            frontPoseEstimate = frontPoseEstimateOptional.get();
        }
        // backPoseEstimator.update(backResult);
        
        // SmartDashboard.putNumber("Apriltag X", apriltagX);
        // SmartDashboard.putNumber("Apriltag Y", apriltagY);
        // SmartDashboard.putNumber("Apriltag Z", apriltagZ);
        // SmartDashboard.putBoolean("Front Camera Has Target", frontCameraHasTarget());
    }

    @Override
    public void simulationPeriodic() {
    }
 
    public PhotonTrackedTarget getVisibleAprilTag(int id) {
        for (int i = 0; i < frontResult.getTargets().size(); i++) {
             if (frontResult.getTargets().get(i).getFiducialId() == id) {
                return frontResult.getTargets().get(i);
             }
        }
        return null;
    }

    public int getFrontAprilTagID() {
    return frontCameraBestTarget.getFiducialId();
    }

    public int getBackAprilTagID() {
    return backCameraBestTarget.getFiducialId();
    }

    public Pose2d getFrontAprilTagPoseInRobotSpace() {
        if (frontCameraBestTarget == null) {
            return new Pose2d();
        }
        Transform3d targetTransform = frontCameraBestTarget.getBestCameraToTarget();
        Translation2d targetTranslation = new Translation2d(targetTransform.getX(), targetTransform.getY());

        Rotation2d targetRotation = new Rotation2d(targetTransform.getRotation().getZ());

        return new Pose2d(targetTranslation, targetRotation);
    }

    public Pose2d getBackAprilTagPoseInRobotSpace() {
        if (frontCameraBestTarget == null) {
            return new Pose2d();
        }
        Transform3d targetTransform = backCameraBestTarget.getBestCameraToTarget();
        Translation2d targetTranslation = new Translation2d(targetTransform.getX(), targetTransform.getY());

        Rotation2d targetRotation = new Rotation2d(targetTransform.getRotation().getZ());

        return new Pose2d(targetTranslation, targetRotation);
    }

    public Pose2d getAprilTagPoseInRobotSpace(int id) {
        Transform3d targetTransform = getVisibleAprilTag(id).getBestCameraToTarget();
        Translation2d targetTranslation = new Translation2d(targetTransform.getX(), targetTransform.getY());

        Rotation2d targetRotation = new Rotation2d(targetTransform.getRotation().getZ());

        return new Pose2d(targetTranslation, targetRotation);
    }

    public Rotation2d getFrontAprilTagYaw() {
        if (frontCameraBestTarget == null) {
            return new Rotation2d();
        }

        return new Rotation2d(Math.toRadians(frontCameraBestTarget.getYaw()));
    }

    // public Pose2d getBackAprilTagPoseInRobotSpace() {
    // Transform3d targetTransform = backCameraTarget.getBestCameraToTarget();
    // Translation2d targetTranslation = new Translation2d(targetTransform.getX(),
    // targetTransform.getY());

    // Rotation2d targetRotation = new Rotation2d(backCameraTarget.getYaw());

    // return new Pose2d(targetTranslation, targetRotation);
    // }

    public boolean frontCameraHasTarget() {
        return frontCameraHasTarget;
    }

    // public boolean backCameraHasTarget() {
    // return backCameraResult.hasTargets();
    // }

    // -------------------------------------------------------------------------------
    // Pose Estimation:
    // --------------------------------------------------------------------------------

    public Pose2d getFrontPoseEstimate2d() {
        return frontPoseEstimate.estimatedPose.toPose2d();
    }

    public double getFrontPoseEstimateTimestamp() {
        return frontPoseEstimate.timestampSeconds;
    }
}
