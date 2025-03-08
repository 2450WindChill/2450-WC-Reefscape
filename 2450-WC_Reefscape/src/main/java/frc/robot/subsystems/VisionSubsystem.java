package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public VisionSubsystem() {
        double dummyX = 0.0;
        double dummyY = 0.0;
        double dummyZ = 0.0;

        tab = Shuffleboard.getTab("Default");
        tab.add("Apriltag X", dummyX);
        tab.add("Apriltag Y", dummyY);
        tab.add("Apriltag Z", dummyZ);

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
                // for (var target : result.getTargets()) {
                //     if (target.getFiducialId() == 1) {
                        frontCameraBestTarget = frontResult.getBestTarget();
                        apriltagX = getFrontAprilTagPoseInRobotSpace().getX();
                        apriltagY = getFrontAprilTagPoseInRobotSpace().getY();
                        apriltagZ = getFrontAprilTagPoseInRobotSpace().getRotation().getDegrees();
                    // }
                // }
            }
            frontCameraHasTarget = frontResult.hasTargets();
        }

        // if (!backCameraResults.isEmpty()) {
        //     // Camera processed a new frame since last
        //     // Get the last one in the list.
        //     backResult = backCameraResults.get(backCameraResults.size() - 1);
        //     if (backResult.hasTargets()) {
        //         // At least one AprilTag was seen by the camera
        //         // for (var target : result.getTargets()) {
        //         //     if (target.getFiducialId() == 1) {
        //                 backCameraBestTarget = frontResult.getBestTarget();
        //             // }
        //         // }
        //     }
        //     frontCameraHasTarget = frontResult.hasTargets();
        // }
        
        SmartDashboard.putNumber("Apriltag X", apriltagX);
        SmartDashboard.putNumber("Apriltag Y", apriltagY);
        SmartDashboard.putNumber("Apriltag Z", apriltagZ);
        SmartDashboard.putBoolean("Front Camera Has Target", frontCameraHasTarget());
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
