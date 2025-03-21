// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Camera;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.CurrentBot;
import frc.robot.commands.AlignToAprilTagParallel;
import frc.robot.commands.AlignToAprilTagSequential;
import frc.robot.commands.ApproachAprilTag;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SquareToAprilTag;
import frc.robot.commands.StrafeToAprilTag;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.SwerveMode;
import frc.robot.Constants.autoConstants.ReefDirection;
import frc.robot.Constants.autoConstants.ReefLevel;
import frc.robot.commands.AlignToAprilTagSequential;
import frc.robot.commands.BopAlgae;
import frc.robot.commands.ClimberMovement;
import frc.robot.commands.CoralIntakeStage1;
import frc.robot.commands.CoralOuttake;
import frc.robot.commands.DeepClimbCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorMovement;
import frc.robot.commands.FullCoralIntake;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DeepClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Vector;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  public CoralSubsystem m_coralSubsystem = null;
  public final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  public DeepClimbSubsystem m_deepClimbSubsystem = null;
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(SwerveMode.KRAKEN,
      m_visionSubsystem);
  public EndEffectorSubsystem m_EndEffectorSubsystem = null;

  private final XboxController m_driverController = new XboxController(ControllerConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(ControllerConstants.kOperatorControllerPort);

  public final JoystickButton dr_aButton = new JoystickButton(m_driverController, Button.kA.value);
  public final JoystickButton dr_bButton = new JoystickButton(m_driverController, Button.kB.value);
  public final JoystickButton dr_xButton = new JoystickButton(m_driverController, Button.kX.value);
  public final JoystickButton dr_yButton = new JoystickButton(m_driverController, Button.kY.value);

  public final JoystickButton dr_leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
  public final JoystickButton dr_rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
  public final JoystickButton dr_startButton = new JoystickButton(m_driverController, Button.kStart.value);

  public final JoystickButton op_aButton = new JoystickButton(m_operatorController, Button.kA.value);
  public final JoystickButton op_bButton = new JoystickButton(m_operatorController, Button.kB.value);
  public final JoystickButton op_xButton = new JoystickButton(m_operatorController, Button.kX.value);
  public final JoystickButton op_yButton = new JoystickButton(m_operatorController, Button.kY.value);

  public final POVButton op_UpDpad = new POVButton(m_operatorController, 0);
  public final POVButton op_DownDpad = new POVButton(m_operatorController, 180);
  public final POVButton op_LeftDpad = new POVButton(m_operatorController, 270);
  public final POVButton op_RightDpad = new POVButton(m_operatorController, 90);

  public final JoystickButton op_leftBumper = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
  public final JoystickButton op_rightBumper = new JoystickButton(m_operatorController, Button.kRightBumper.value);

  private final CurrentBot currentBotState = CurrentBot.COMP;

  public SendableChooser<Command> m_chooser;

  Timer timer = new Timer();
  double time = 0.0;
  private Command Back_Up_Auto;
  private Command One_Meter_Path;
  private Command Algae_Coral_Auto;

  public RobotContainer() {
    if (currentBotState == CurrentBot.COMP) {
      m_coralSubsystem = new CoralSubsystem();
      m_coralSubsystem.setAllianceColor();
      m_deepClimbSubsystem = new DeepClimbSubsystem();
      m_EndEffectorSubsystem = new EndEffectorSubsystem();
    }
    m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> (m_driverController.getLeftY()),
            () -> (m_driverController.getLeftX()),
            () -> (m_driverController.getRightX()),
            () -> Constants.isRobotCentric,
            () -> dr_leftBumper.getAsBoolean(),
            () -> m_driverController.getPOV()));
    configureControllerBindings();
    configureAutoChooser();
    configureDashboardBindings();
  }

  private void configureControllerBindings() {
    // new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5).onTrue(new
    // AlignToAprilTagSequential(
    // m_visionSubsystem, m_drivetrainSubsystem,
    // -Constants.VisionConstants.postOffset, 0.7, Camera.FRONT,
    // () -> (dr_startButton.getAsBoolean()), 4));

    // new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5).onTrue(new
    // AlignToAprilTagSequential(
    // m_visionSubsystem, m_drivetrainSubsystem,
    // -Constants.VisionConstants.postOffset, 0.7, Camera.FRONT,
    // () -> (dr_startButton.getAsBoolean()), 4));

    // Driver Bindings
    dr_aButton.onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro()));
    //dr_bButton.onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroPose()));

    // dr_xButton.onTrue(
        // new MoveToPose(m_drivetrainSubsystem, new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), () -> dr_bButton.getAsBoolean()));
    //dr_bButton.onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.resetPose(new Pose2d(14.81, 1.46, new Rotation2d(Math.toRadians(118))))));
    dr_bButton.onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.resetPose(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))))));
    dr_xButton.onTrue(new MoveToPose(m_drivetrainSubsystem, new Pose2d(14.81, 1.46, new Rotation2d(Math.toRadians(118))), () -> dr_bButton.getAsBoolean()));

    dr_leftBumper.onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.resetMods()));

    // dr_xButton.onTrue(new AlignToAprilTagSequential(m_visionSubsystem,
    // m_drivetrainSubsystem,
    // -Constants.VisionConstants.postOffset, 0.7, Camera.FRONT, () ->
    // (dr_startButton.getAsBoolean()), 4));
    // dr_bButton.onTrue(new AlignToAprilTagSequential(m_visionSubsystem,
    // m_drivetrainSubsystem,
    // Constants.VisionConstants.postOffset, 0.7, Camera.FRONT, () ->
    // (dr_startButton.getAsBoolean()), 4));

    // Only use operator buttons if using the comp robot
    if (currentBotState == CurrentBot.COMP) {
      dr_yButton.onTrue(new DeepClimbCommand(m_deepClimbSubsystem, 0.601, 0.099,  () -> dr_bButton.getAsBoolean()));

      // Operator Bindings
      op_aButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem,
          Constants.intakeHeight));
      op_xButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem,
          Constants.L1Height));
      op_yButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem,
          Constants.L2Height));
      op_bButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem,
          Constants.L3Height));
      op_rightBumper.onTrue(new CoralOuttake(m_coralSubsystem, m_EndEffectorSubsystem, 0.2));
      op_RightDpad.onTrue(new CoralOuttake(m_coralSubsystem, m_EndEffectorSubsystem, 0.03));
      op_leftBumper.onTrue(new FullCoralIntake(m_coralSubsystem, m_EndEffectorSubsystem,  0.2, 0.25));

      dr_leftBumper.whileTrue(new ClimberMovement(m_deepClimbSubsystem, "out", 0.05));
      dr_rightBumper.whileTrue(new ClimberMovement(m_deepClimbSubsystem, "in", 0.05));

      dr_yButton.onTrue(new DeepClimbCommand(m_deepClimbSubsystem, 0.099, 0.601,  () -> dr_bButton.getAsBoolean()));
      
      // ELEVATOR COMMANDS COMMENTED OUT FOR NOW
      op_UpDpad.whileTrue(new ElevatorMovement(m_coralSubsystem, "up", 0.05));
      op_DownDpad.whileTrue(new ElevatorMovement(m_coralSubsystem, "down", 0.05));

      m_EndEffectorSubsystem.setDefaultCommand(
          new BopAlgae(
              m_EndEffectorSubsystem,
              () -> (m_operatorController.getRightTriggerAxis()) * 0.5,
              () -> (m_operatorController.getLeftTriggerAxis()) * 0.5));
    }
  }

  private Command intakeSequence() {
    return Commands.runOnce(() -> new MoveElevatorToPosition(m_coralSubsystem, Constants.intakeHeight))
        .andThen(new CoralIntakeStage1(m_coralSubsystem, m_EndEffectorSubsystem, 0.1));
  }

  private void configureDashboardBindings() {

    if (currentBotState == CurrentBot.TEST) {
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Default");
    tab.add("SquareToAprilTag",
        new SquareToAprilTag(m_visionSubsystem, m_drivetrainSubsystem, Camera.FRONT,
            () -> (dr_startButton.getAsBoolean())))
        .withWidget(BuiltInWidgets.kCommand);
    tab.add("StrafeToAprilTag",
        new StrafeToAprilTag(m_visionSubsystem, m_drivetrainSubsystem, 0, Camera.FRONT,
            () -> (dr_startButton.getAsBoolean())))
        .withWidget(BuiltInWidgets.kCommand);
    tab.add("ApproachAprilTag",
        new ApproachAprilTag(m_visionSubsystem, m_drivetrainSubsystem, 1.3, Camera.FRONT,
            () -> (dr_startButton.getAsBoolean())))
        .withWidget(BuiltInWidgets.kCommand);
    tab.add("AlignToAprilTag",
        new AlignToAprilTagSequential(m_visionSubsystem, m_drivetrainSubsystem, 0, 1.3, Camera.FRONT,
            () -> (dr_startButton.getAsBoolean()), 4))
        .withWidget(BuiltInWidgets.kCommand);

    tab.add("Intake height", new MoveElevatorToPosition(m_coralSubsystem, -25)).withWidget(BuiltInWidgets.kCommand);
    tab.add("L1 height", new MoveElevatorToPosition(m_coralSubsystem, Constants.L1Height))
        .withWidget(BuiltInWidgets.kCommand);
    tab.add("L2 height", new MoveElevatorToPosition(m_coralSubsystem, Constants.L2Height))
        .withWidget(BuiltInWidgets.kCommand);
    tab.add("L3 height", new MoveElevatorToPosition(m_coralSubsystem, Constants.L3Height))
        .withWidget(BuiltInWidgets.kCommand);
  }

  // Basic auto for testing, backs up after a certain period of time
  // private Command autoBackUp() {
  // return new MoveElevatorToPosition(m_coralSubsystem, Constants.intakeHeight)
  // .andThen(new CoralIntake(m_coralSubsystem, 0.1))
  // .andThen(Commands.runOnce(() -> m_drivetrainSubsystem.drive(new
  // Translation2d(-1, 0),
  // m_drivetrainSubsystem.gyro.getYaw().getValueAsDouble(), true, false),
  // m_drivetrainSubsystem))
  // .andThen((new WaitCommand(5)))
  // .andThen(Commands.runOnce(() -> m_drivetrainSubsystem.drive(new
  // Translation2d(0, 0),
  // m_drivetrainSubsystem.gyro.getYaw().getValueAsDouble(), true, false)));
  // //.andThen(scoreCoral(ReefDirection.LEFT, ReefLevel.L2));
  // }

  // AUTO NO ELEVATOR
  private Command autoBackUp() {
    return Commands.runOnce(() -> m_drivetrainSubsystem.drive(new Translation2d(-1, 0),
        m_drivetrainSubsystem.gyro.getYaw().getValueAsDouble(), true, false), m_drivetrainSubsystem)
        .andThen((new WaitCommand(5)))
        .andThen(Commands.runOnce(() -> m_drivetrainSubsystem.drive(new Translation2d(0, 0),
            m_drivetrainSubsystem.gyro.getYaw().getValueAsDouble(), true, false)));
  }

  private Command scoreCoral(ReefDirection direction, ReefLevel level) {
    double strafeOffset = Constants.VisionConstants.postOffset;
    double height;
    if (direction == ReefDirection.LEFT) {
      strafeOffset = strafeOffset * -1;
    }

    switch (level) {
      case L1:
        height = Constants.L1Height;
      case L2:
        height = Constants.L2Height;
      case L3:
        height = Constants.L3Height;
      default:
        height = Constants.L1Height;
    }

    return Commands.parallel(
        new AlignToAprilTagSequential(m_visionSubsystem, m_drivetrainSubsystem,
            strafeOffset, 2, Camera.FRONT, () -> (dr_startButton.getAsBoolean()), 2),
        new MoveElevatorToPosition(m_coralSubsystem, height))
        .andThen(new CoralOuttake(m_coralSubsystem, m_EndEffectorSubsystem, 0.5));
  }

  private Command intakePreLoad() {
    return new MoveElevatorToPosition(m_coralSubsystem, Constants.intakeHeight)
        .andThen(new FullCoralIntake(m_coralSubsystem, m_EndEffectorSubsystem,  0.2, 0.25));
  }

  private void configureAutoChooser() {
    m_chooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Mode", m_chooser);
    // m_chooser.addOption("Autonomous", scoreCoral(ReefDirection.LEFT,
    // ReefLevel.L2));
  }

  // Auto command
  public Command getAutonomousCommand() {
    return autoBackUp();
    // return scoreCoral(ReefDirection.LEFT, ReefLevel.L2);
    // return m_chooser.getSelected();
    // return new InstantCommand();
    // return Commands.runOnce(() -> m_drivetrainSubsystem.drive(new
    // Translation2d(0, 0.5),
    // m_drivetrainSubsystem.gyro.getYaw().getValueAsDouble(),
    // false,
    // false))
    // .andThen(new WaitCommand(4))
    // .andThen(Commands.runOnce(() -> m_drivetrainSubsystem.drive(new
    // Translation2d(0, 0),
    // m_drivetrainSubsystem.gyro.getYaw().getValueAsDouble(),
    // false,
    // false))
    // // .andThen(intakePreLoad())
    // .andThen(scoreCoral(ReefDirection.LEFT, ReefLevel.L2)));
  }

  
}