package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera frontCamera = new PhotonCamera("frontCamera");

    public VisionSubsystem() {
        
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> frontCameraResults = frontCamera.getAllUnreadResults();
    }

    @Override
    public void simulationPeriodic() {
    }

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

    // TODO: Convert to photonvision equivalent

    // Gets the ID of the primary apriltag in the limelights view
    // public double getAprilTagID() {
    // return LimelightHelpers.getFiducialID("limelight");
    // }

    // TODO: Convert to photonvision equivalent
    
    // Get the 3d bot pose of the primary apriltag in the limelights view relative
    // to the robots pose
    // public Pose3d getAprilTagPoseToBot3d() {
    // return LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    // }

    // TODO: Convert to photonvision equivalent

    // Get the 3d pose of the targetted apriltag realtive to the robot
    // public Pose3d getBotPoseToAprilTag() {
    // return LimelightHelpers.getBotPose3d_TargetSpace("limelight");
    // }

    // TODO: Convert to photonvision equivalent

    // Get the 2d bot pose of the primary apriltag in the limelights view relative
    // to the robots pose
    // public Pose2d getAprilTagPoseToBot2d() {
    // return new Pose2d(getAprilTagPoseToBot3d().getX(),
    // getAprilTagPoseToBot3d().getY(),
    // getAprilTagPoseToBot3d().getRotation().toRotation2d());
    // }

    // TODO: Convert to photonvision equivalent
    
    // Gets distance to primary april tag
    // Returns in meters
    // public double getDistanceToAprilTag2d() {
    // return Math.sqrt(
    // Math.pow(getAprilTagPoseToBot2d().getX(), 2)
    // + Math.pow(getAprilTagPoseToBot2d().getY(), 2)
    // );
    // }
}
