package frc.robot.swerveModules;

import java.sql.Driver;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.libs.OnboardModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WindChillKrakenSwerveModule extends BaseWindChillSwerveModule{
  public int moduleNumber;
  private Rotation2d lastAngle;

  private SparkMax angleMotor;
  private TalonFX driveMotor;
  private InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
  private NeutralModeValue idleMode = NeutralModeValue.Brake;
  private double PeakForwardVoltage = 12.0;
  private double PeakReverseVoltage = -12.0;
  

  private SparkBaseConfig angleConfig;
  private TalonFXConfiguration talonFXConfigs;
  private CurrentLimitsConfigs currentConfigs;
  private VoltageConfigs voltageConfigs;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  // private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;

  public WindChillKrakenSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    super(moduleNumber);
    this.moduleNumber = moduleNumber;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, "canivore");

    /* Angle Motor Config */
    angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getClosedLoopController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.driveMotorID);

    configDriveMotor();
    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isSlowMode) {
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isSlowMode);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isSlowMode) {
    double percentOutput = desiredState.speedMetersPerSecond / Constants.maxSpeed;
    if (!isSlowMode) {
      driveMotor.set(percentOutput);
    } else {
      driveMotor.set(percentOutput * 0.15);
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    double desiredAngle = angle.getDegrees();

    // while (desiredAngle < 0) {
    // desiredAngle += 360;
    // }

    // while (desiredAngle > 360) {
    // desiredAngle -= 360;
    // }\[]
    

    SmartDashboard.putNumber("Desired Angle " + moduleNumber, desiredAngle);

    angleController.setReference(desiredAngle, SparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoderInDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  // Using getCancoder instead
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  private Rotation2d getAutoAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public double getCanCoderInDegrees() {
    return getRawCanCoder() * 360.0;
  }

  public double getRawCanCoder() {
    return angleEncoder.getAbsolutePosition().getValueAsDouble();
  }

  // TODO: Once we get Kraken encoder this should work (done?), make sure where these methods are used the types are correct
  public double getDriveEncoder() {
    return driveMotor.getPosition().getValueAsDouble() * Constants.rotationsPerOneMeter;
  }

  // TODO: Once we get Kraken encoder this should work (done?)
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), getAngle());
  }

  // TODO: Once we get Kraken encoder this should work (done?)
  public SwerveModuleState getAutoState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), getAutoAngle());
  }

  private void configAngleMotor() {

    angleConfig = new SparkMaxConfig();
    angleConfig
        .smartCurrentLimit(Constants.angleContinuousCurrentLimit)
        .inverted(Constants.angleInvert)
        .voltageCompensation(Constants.voltageComp);
    angleConfig.encoder
        .positionConversionFactor(Constants.angleConversionFactor);
    angleConfig.closedLoop
        // Wrap angle motors to stay within 0-360 degrees
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0.0)
        .positionWrappingMaxInput(360.0)
        .pidf(Constants.angleKP, Constants.angleKI, Constants.angleKD, Constants.angleKFF);

    angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // TODO: Need to figure out kraken version of configs
  private void configDriveMotor() {

    var talonFXConfigs = new TalonFXConfiguration();
    // var motorConfigs = new MotorOutputConfigs();
    // var voltageConfigs = new VoltageConfigs();

    // enable stator current limit
    // var limitConfigs = new CurrentLimitsConfigs();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 30;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    // TODO: Look at this current limiting code compared to what you have
    // talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
    // talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.driveContinuousCurrentLimit;
    // talonFXConfigs.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
    // talonFXConfigs.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;
  
    talonFXConfigs.MotorOutput.Inverted = invertedValue;
    talonFXConfigs.MotorOutput.NeutralMode = idleMode;
    talonFXConfigs.Feedback.RotorToSensorRatio = Constants.driveGearRatio;
    // in init function, set slot 0 gains
    // var slot0Configs = new Slot0Configs();
  
    // Setting PID values
    talonFXConfigs.Slot0.kP = 0.11;
    talonFXConfigs.Slot0.kI = 0;
    talonFXConfigs.Slot0.kD = 0;

    /* Gear Ratio Config */
    // Applying configs to the configurator
    // driveMotor.getConfigurator().apply(limitConfigs);
    // driveMotor.getConfigurator().apply(motorConfigs);
    // driveMotor.getConfigurator().apply(slot0Configs);
    driveMotor.getConfigurator().apply(talonFXConfigs);
  }

  // public SwerveModulePosition getPosition() {
  // return new SwerveModulePosition(
  // -((getDriveEncoder() / Constants.rotationsPerOneFoot) *
  // Constants.feetToMeters),
  // Rotation2d.fromDegrees(getCanCoderInDegrees()));
  // }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        -((getDriveEncoder() / Constants.rotationsPerOneMeter) * Constants.feetToMeters) * 0.9,
        Rotation2d.fromDegrees(getCanCoderInDegrees()));
  }

  public void setPosition(double position) {
    integratedAngleEncoder.setPosition(position);
  }
}
