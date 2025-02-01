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
    // /PhotonCamera backCamera = new PhotonCamera("backCamera");

    List<PhotonPipelineResult> frontCameraResults;
    PhotonPipelineResult backCameraResult;

    PhotonTrackedTarget frontCameraTarget;
    PhotonTrackedTarget backCameraTarget;

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
        double apriltagX = 0.0;
        double apriltagY = 0.0;
        double apriltagZ = 0.0;
        frontCameraResults = frontCamera.getAllUnreadResults();
        boolean targetVisible = false;

        frontCameraTarget = null;

        if (!frontCameraResults.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = frontCameraResults.get(frontCameraResults.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 1) {
                        frontCameraTarget = target;
                        apriltagX = getFrontAprilTagPoseInRobotSpace().getX();
                        apriltagY = getFrontAprilTagPoseInRobotSpace().getY();
                        apriltagZ = getFrontAprilTagPoseInRobotSpace().getRotation().getRadians();
                    }
                }
            }
        }
        
        SmartDashboard.putNumber("Apriltag X", apriltagX);
        SmartDashboard.putNumber("Apriltag Y", apriltagY);
        SmartDashboard.putNumber("Apriltag Z", apriltagZ);
    }

    @Override
    public void simulationPeriodic() {
    }

    // public int getFrontAprilTagID() {
    // return frontCameraTarget.getFiducialId();
    // }

    // public int getBackAprilTagID() {
    // return backCameraTarget.getFiducialId();
    // }

    public Pose2d getFrontAprilTagPoseInRobotSpace() {
        if (frontCameraTarget == null) {
            return new Pose2d();
        }
        Transform3d targetTransform = frontCameraTarget.getBestCameraToTarget();
        Translation2d targetTranslation = new Translation2d(targetTransform.getX(), targetTransform.getY());

        Rotation2d targetRotation = new Rotation2d(frontCameraTarget.getYaw());

        return new Pose2d(targetTranslation, targetRotation);
    }

    // public Pose2d getBackAprilTagPoseInRobotSpace() {
    // Transform3d targetTransform = backCameraTarget.getBestCameraToTarget();
    // Translation2d targetTranslation = new Translation2d(targetTransform.getX(),
    // targetTransform.getY());

    // Rotation2d targetRotation = new Rotation2d(backCameraTarget.getYaw());

    // return new Pose2d(targetTranslation, targetRotation);
    // }

    public boolean frontCameraHasTarget() {
        return frontCameraTarget != null;
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
