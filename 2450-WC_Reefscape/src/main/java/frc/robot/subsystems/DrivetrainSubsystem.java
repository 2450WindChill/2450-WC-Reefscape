package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.WindChillSwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  public final Pigeon2 gyro;
  public WindChillSwerveModule[] swerveModules;
  public SwerveDriveOdometry swerveOdometry;
  public SparkMax testMotor;
  public CANcoder canCoder;
  public HolonomicDriveController holonomicDriveController;
  public static SwerveDrivePoseEstimator poseEstimate;

  public DrivetrainSubsystem() {
    holonomicDriveController = new HolonomicDriveController(
        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14)));

    gyro = new Pigeon2(Constants.pigeonID, "canivore");
    zeroGyro();

    // TODO: Implement kraken motor controllers to swerve modules
    swerveModules = new WindChillSwerveModule[] {
        new WindChillSwerveModule(0, Constants.FrontLeftModule.constants),
        new WindChillSwerveModule(1, Constants.FrontRightModule.constants),
        new WindChillSwerveModule(2, Constants.BackLeftModule.constants),
        new WindChillSwerveModule(3, Constants.BackRightModule.constants)
    };
  }

  @Override
  public void periodic() {
  }

  // --------------------------------------------------------------------------------
  // Swerve instantiation and methods 
  // --------------------------------------------------------------------------------
  public void drive(Translation2d translation, double rotation, boolean isRobotCentric, boolean isSlowMode) {
    SwerveModuleState[] swerveModuleStates;
    if (isRobotCentric) {
      swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    } else {
      swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(),
              translation.getY(),
              rotation,
              getGyroYaw()));
    }

    for (WindChillSwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isSlowMode);
    }
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPosition();
    }
    return positions;
  }

  public Rotation2d getGyroYaw() {
    Rotation2d rotation2d = Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    return rotation2d;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (WindChillSwerveModule mod : swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (WindChillSwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModuleState[] getAutoModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (WindChillSwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getAutoState();
    }
    return states;
  }

  public WindChillSwerveModule[] getModules() {
    return swerveModules;
  }

  public void resetMods() {
    for (WindChillSwerveModule mod : swerveModules) {
      mod.resetToAbsolute();
    }
  }

  public double getFrontLeftEncoderVal() {
    double frontLeftEncoderVal = swerveModules[0].getDriveEncoder();

    return frontLeftEncoderVal;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public void setGyro(int angle) {
    gyro.setYaw(angle);
  }

  // -------------------------------------------------------------------------------
  // Pose Estimation:
  // --------------------------------------------------------------------------------

    // Gets current estimated bot pose
    public static Pose2d getBotPose() {
        return poseEstimate.getEstimatedPosition();
    }

    public Pose2d getThisPose() {
        return poseEstimate.getEstimatedPosition();
    }

    public void zeroPose() {
        poseEstimate = new SwerveDrivePoseEstimator(
                Constants.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(0, 0, new Rotation2d(0)));
    }

    public void resetPose(Pose2d newPose) {
        poseEstimate.resetPosition(gyro.getRotation2d(), getPositions(),
                newPose);
    }

    // Gets estimated bot x
    public double getBotX() {
        return getBotPose().getX();
    }

    // Gets estimated bot y
    public double getBotY() {
        return getBotPose().getY();
    }

    // Gets estimated bot rotation
    public double getBotRotation() {
        return getBotPose().getRotation().getDegrees();
    }

  // -------------------------------------------------------------------------------
  // Pathplanning methods:
  // --------------------------------------------------------------------------------

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public StatusCode resetCANcoder() {
    return canCoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetCANcoder();
    swerveOdometry.resetPosition(
        gyro.getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getAutoSpeeds() {
    ChassisSpeeds temChassisSpeeds = Constants.swerveKinematics.toChassisSpeeds(getAutoModuleStates());
    ChassisSpeeds reversedcChassisSpeeds = new ChassisSpeeds(-temChassisSpeeds.vxMetersPerSecond, -temChassisSpeeds.vyMetersPerSecond, -temChassisSpeeds.omegaRadiansPerSecond);
    return reversedcChassisSpeeds;
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.maxSpeed);
    var modules = getModules();
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(targetStates[i], false);
    }
  }

  public void setPosition(double position) {
    swerveModules[0].setPosition(position);
    System.err.println("Setting position to 0");
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds reversedChassisSpeeds = new ChassisSpeeds(-robotRelativeSpeeds.vxMetersPerSecond, -robotRelativeSpeeds.vyMetersPerSecond, -robotRelativeSpeeds.omegaRadiansPerSecond);
    SwerveModuleState[] modStates = Constants.swerveKinematics.toSwerveModuleStates(reversedChassisSpeeds);

    for (WindChillSwerveModule mod : swerveModules) {
      mod.setDesiredState(modStates[mod.moduleNumber], false);
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}