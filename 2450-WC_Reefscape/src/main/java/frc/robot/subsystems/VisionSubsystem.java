package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera frontCamera = new PhotonCamera("frontCamera");
    // /PhotonCamera backCamera = new PhotonCamera("backCamera");

    List<PhotonPipelineResult> frontCameraResults;
    PhotonPipelineResult backCameraResult;

    PhotonTrackedTarget frontCameraTarget;
    PhotonTrackedTarget backCameraTarget;


    public VisionSubsystem() {
        
    }

    @Override
    public void periodic() {
         // Read in relevant data from the Camera
         double apriltagX = 0.0;
         double apriltagY = 0.0;
         double apriltagZ = 0.0;
         frontCameraResults = frontCamera.getAllUnreadResults();
         boolean targetVisible = false;

         if (!frontCameraResults.isEmpty()) {
             // Camera processed a new frame since last
             // Get the last one in the list.
             var result = frontCameraResults.get(frontCameraResults.size() - 1);
             if (result.hasTargets()) {
                 // At least one AprilTag was seen by the camera
                 for (var target : result.getTargets()) {
                     if (target.getFiducialId() == 1) {
                         apriltagX = getFrontAprilTagPoseInRobotSpace().getX();
                         apriltagY = getFrontAprilTagPoseInRobotSpace().getY();
                         apriltagZ =  getFrontAprilTagPoseInRobotSpace().getRotation().getRadians();
                     }
                 }
             }
         }

        // frontCameraResults = frontCamera.getAllUnreadResults();

        // if (frontCameraResults.isEmpty()) {
        //     return;
        // }

        // frontCameraTarget =  frontCameraResults.get(0).getBestTarget();

        // if (frontCameraTarget == null) {
        //     return;
        // }

        // if(backCamerahasTarget()) {
        //     backCameraResult = backCamera.getAllUnreadResults().get(backCamera.getAllUnreadResults().size() - 1);
        //     backCameraTarget = backCameraResult.getBestTarget();
        // }

        SmartDashboard.putNumber("Front AprilTag X", apriltagX);
        SmartDashboard.putNumber("Front AprilTag Y", apriltagY);
        SmartDashboard.putNumber("Front AprilTag Rotation", apriltagZ);
    }

    @Override
    public void simulationPeriodic() {
    }

    // public int getFrontAprilTagID() {
    //     return frontCameraTarget.getFiducialId();
    // }

    // public int getBackAprilTagID() {
    //     return backCameraTarget.getFiducialId();
    // }
    
    public Pose2d getFrontAprilTagPoseInRobotSpace() {
            Transform3d targetTransform = frontCameraTarget.getBestCameraToTarget();
            Translation2d targetTranslation = new Translation2d(targetTransform.getX(), targetTransform.getY());

            Rotation2d targetRotation = new Rotation2d(frontCameraTarget.getYaw());

            return new Pose2d(targetTranslation, targetRotation);
    }

    // public Pose2d getBackAprilTagPoseInRobotSpace() {
    //     Transform3d targetTransform = backCameraTarget.getBestCameraToTarget();
    //     Translation2d targetTranslation = new Translation2d(targetTransform.getX(), targetTransform.getY());

    //     Rotation2d targetRotation = new Rotation2d(backCameraTarget.getYaw());

    //     return new Pose2d(targetTranslation, targetRotation);
    // }

    public boolean frontCameraHasTarget() {
        System.out.println(frontCamera.getAllUnreadResults().size());
        if (frontCamera.getAllUnreadResults().size() != 0) {
            return frontCamera.getAllUnreadResults().get(frontCamera.getAllUnreadResults().size()).hasTargets();
        } else {
            return false;
        }
    }

    // public boolean backCameraHasTarget() {
    //     return backCameraResult.hasTargets();
    // }

    // -------------------------------------------------------------------------------
    // Pose Estimation:
    // --------------------------------------------------------------------------------

    // TODO: Convert to photonvision equivalent

    // Gets bot pose using limelight always relative to blue alliance wall
    // public Pose2d getLimelightPose() {
    // return LimelightHelpers.getBotPose2d("limelight");
    // }

    // TODO: Convert to photonvision equivalent

    // Gets gametime and then subtracts the latency of the pose
    // public double getTimeStamp() {
    // return Timer.getFPGATimestamp()
    // - ((LimelightHelpers.getLatency_Capture("limelight")
    // + LimelightHelpers.getLatency_Pipeline("limelight"))
    // / 1000.0);
    // }
}
