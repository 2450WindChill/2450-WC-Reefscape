package frc.robot.subsystems;

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
    PhotonCamera backCamera = new PhotonCamera("backCamera");

    PhotonPipelineResult frontCameraResult;
    PhotonPipelineResult backCameraResult;

    PhotonTrackedTarget frontCameraTarget;
    PhotonTrackedTarget backCameraTarget;


    public VisionSubsystem() {
        
    }

    @Override
    public void periodic() {
        frontCameraResult = frontCamera.getAllUnreadResults().get(frontCamera.getAllUnreadResults().size() - 1);
        backCameraResult = backCamera.getAllUnreadResults().get(backCamera.getAllUnreadResults().size() - 1);

        frontCameraTarget = frontCameraResult.getBestTarget();
        backCameraTarget = backCameraResult.getBestTarget();

        SmartDashboard.putNumber("Front AprilTag X", getFrontAprilTagPoseInRobotSpace().getX());
        SmartDashboard.putNumber("Front AprilTag Y", getFrontAprilTagPoseInRobotSpace().getY());
        SmartDashboard.putNumber("Front AprilTag Rotation", getFrontAprilTagPoseInRobotSpace().getRotation().getRadians());
    }

    @Override
    public void simulationPeriodic() {
    }

    public int getFrontAprilTagID() {
        return frontCameraTarget.getFiducialId();
    }

    public int getBackAprilTagID() {
        return backCameraTarget.getFiducialId();
    }
    
    public Pose2d getFrontAprilTagPoseInRobotSpace() {
        Transform3d targetTransform = frontCameraTarget.getBestCameraToTarget();
        Translation2d targetTranslation = new Translation2d(targetTransform.getX(), targetTransform.getY());

        Rotation2d targetRotation = new Rotation2d(frontCameraTarget.getYaw());

        return new Pose2d(targetTranslation, targetRotation);
    }

    public Pose2d getBackAprilTagPoseInRobotSpace() {
        Transform3d targetTransform = backCameraTarget.getBestCameraToTarget();
        Translation2d targetTranslation = new Translation2d(targetTransform.getX(), targetTransform.getY());

        Rotation2d targetRotation = new Rotation2d(backCameraTarget.getYaw());

        return new Pose2d(targetTranslation, targetRotation);
    }

    public boolean frontCameraHasTarget() {
        return frontCameraResult.hasTargets();
    }

    public boolean backCameraHasTarget() {
        return backCameraResult.hasTargets();
    }

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
