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
import frc.robot.commands.BopAlgae;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SquareToAprilTag;
import frc.robot.commands.StrafeToAprilTag;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.SwerveMode;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.StartingPosition;
import frc.robot.commands.AlignToAprilTagSequential;
import frc.robot.commands.BopAlgaeWithTriggers;
import frc.robot.commands.ClimberMovement;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralOuttake;
import frc.robot.commands.DeepClimbCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorMovement;
import frc.robot.commands.KillDriveCommands;
import frc.robot.commands.KillOperatorCommands;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  // public DeepClimbSubsystem m_deepClimbSubsystem = null;
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(SwerveMode.KRAKEN,
      m_visionSubsystem);
  public EndEffectorSubsystem m_endEffectorSubsystem = null;
  public final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

  private final XboxController m_driverController = new XboxController(ControllerConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(ControllerConstants.kOperatorControllerPort);

  public final JoystickButton dr_aButton = new JoystickButton(m_driverController, Button.kA.value);
  public final JoystickButton dr_bButton = new JoystickButton(m_driverController, Button.kB.value);
  public final JoystickButton dr_xButton = new JoystickButton(m_driverController, Button.kX.value);
  public final JoystickButton dr_yButton = new JoystickButton(m_driverController, Button.kY.value);
  public final JoystickButton dr_minusButton = new JoystickButton(m_driverController, Button.kBack.value);

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

  public final JoystickButton op_startButton = new JoystickButton(m_operatorController, Button.kStart.value);

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
      // m_deepClimbSubsystem = new DeepClimbSubsystem();
      m_endEffectorSubsystem = new EndEffectorSubsystem();
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
    m_LEDSubsystem.setAllianceColor();

    ShuffleboardTab tab = Shuffleboard.getTab("testing");
  }

  private void configureControllerBindings() {
    // Driver Bindings
    dr_aButton.onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro()));
    dr_bButton.onTrue(Commands
        .runOnce(() -> m_drivetrainSubsystem.resetPose(new Pose2d(13.18, 0.5, new Rotation2d(Math.toRadians(-180))))));
    // dr_xButton.onTrue(new MoveToPose(m_drivetrainSubsystem, new Pose2d(11.9, 2.2, new Rotation2d(Math.toRadians(-180))),
    //     () -> dr_bButton.getAsBoolean()));

    dr_leftBumper.onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.resetMods()));
    dr_minusButton.onTrue(new KillDriveCommands(m_drivetrainSubsystem));

    // Only use operator buttons if using the comp robot
    if (currentBotState == CurrentBot.COMP) {
      // dr_yButton.onTrue(new DeepClimbCommand(m_deepClimbSubsystem, 0.601, 0.099, ()
      // -> dr_bButton.getAsBoolean()));

      // Operator Bindings
      op_aButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem,
          Constants.intakeHeight));
      op_xButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem,
          Constants.L1Height));
      op_yButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem,
          Constants.L2Height));
      op_bButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem,
          Constants.L3Height));
      op_rightBumper.onTrue(new CoralOuttake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2));
      op_RightDpad.onTrue(new CoralOuttake(m_endEffectorSubsystem, m_LEDSubsystem, 0.03));
      op_leftBumper.onTrue(new CoralIntake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2));

      op_startButton.onTrue(new KillOperatorCommands(m_coralSubsystem, m_endEffectorSubsystem));

      // dr_leftBumper.whileTrue(new ClimberMovement(m_deepClimbSubsystem, "out", 0.05));
      // dr_rightBumper.whileTrue(new ClimberMovement(m_deepClimbSubsystem, "in", 0.05));

      // dr_yButton.onTrue(new DeepClimbCommand(m_deepClimbSubsystem, 0.099, 0.601, ()
      // -> dr_bButton.getAsBoolean()));

      op_UpDpad.whileTrue(new ElevatorMovement(m_coralSubsystem, "up", 0.15));
      op_DownDpad.whileTrue(new ElevatorMovement(m_coralSubsystem, "down", 0.15));
      op_LeftDpad.whileTrue(bopLowAlgaeSequence());
      op_RightDpad.whileTrue(bopHighAlgaeSequence());

      m_endEffectorSubsystem.setDefaultCommand(
          new BopAlgaeWithTriggers(
              m_endEffectorSubsystem,
              () -> (m_operatorController.getRightTriggerAxis()) * 0.5,
              () -> (m_operatorController.getLeftTriggerAxis()) * 0.5));
    }
  }

  private Command intakeSequence() {
    return Commands
        .runOnce(() -> new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem,
            Constants.intakeHeight))
        .andThen(new CoralIntake(m_endEffectorSubsystem, m_LEDSubsystem, .1));
  }

  private Command bopLowAlgaeSequence() {
    return Commands.parallel(
        new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.lowBopAlgae),
        new BopAlgae(m_endEffectorSubsystem, 0.2));
  }

  private Command bopHighAlgaeSequence() {
    return Commands.parallel(
        new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.highBopAlgae),
        new BopAlgae(m_endEffectorSubsystem, 0.2));
  }

  private void configureDashboardBindings() {

    if (currentBotState == CurrentBot.TEST) {
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Default");

    tab.add("Intake height", new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, -25))
        .withWidget(BuiltInWidgets.kCommand);
    tab.add("L1 height",
        new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.L1Height))
        .withWidget(BuiltInWidgets.kCommand);
    tab.add("L2 height",
        new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.L2Height))
        .withWidget(BuiltInWidgets.kCommand);
    tab.add("L3 height",
        new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.L3Height))
        .withWidget(BuiltInWidgets.kCommand);
    tab.add("ON", Commands.runOnce(() -> m_LEDSubsystem.setLEDColor(0, 255, 0, 0)));
    tab.add("Flow", Commands.runOnce(() -> m_LEDSubsystem.setLEDSFlowing(0, 0, 255, 0)));
    tab.add("Blink", Commands.runOnce(() -> m_LEDSubsystem.setLEDSBlinking(0, 0, 255, 0)));
    tab.add("OFF", Commands.runOnce(() -> m_LEDSubsystem.setLEDColor(0, 0, 0, 0)));
    tab.add("FIRE", Commands.runOnce(() -> m_LEDSubsystem.fireLEDS()));
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

  private Command oneCoralAuto(StartingPosition startingPosition) {
    Pose2d scoringPose = m_drivetrainSubsystem.getThisPose();
    switch (startingPosition) {
      case redLeft:
        m_drivetrainSubsystem.resetPose(AutoConstants.redLeftStartingPose);
        scoringPose = AutoConstants.redLeftScoringPose;

      case redMiddle:
        m_drivetrainSubsystem.resetPose(AutoConstants.redMiddleStartingPose);
        scoringPose = AutoConstants.redMiddleScoringPose;

      case redRight:
        m_drivetrainSubsystem.resetPose(AutoConstants.redRightStartingPose);
        scoringPose = AutoConstants.redRightScoringPose;

      case blueLeft:
        m_drivetrainSubsystem.resetPose(AutoConstants.blueLeftStartingPose);
        scoringPose = AutoConstants.blueLeftScoringPose;

      case blueMiddle:
        m_drivetrainSubsystem.resetPose(AutoConstants.blueMiddleStartingPose);
        scoringPose = AutoConstants.blueMiddleScoringPose;

      case blueRight:
        m_drivetrainSubsystem.resetPose(AutoConstants.blueRightStartingPose);
        scoringPose = AutoConstants.blueRightScoringPose;
    }


    return Commands.parallel(
      new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.intakeHeight),
      new CoralIntake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2))
      
    .andThen(Commands.parallel(
      new MoveToPose(m_drivetrainSubsystem, scoringPose, dr_aButton, 5),
      new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.L2Height)))

    .andThen(new CoralOuttake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2));
  }

  private void configureAutoChooser() {
    m_chooser = new SendableChooser<>();
    SmartDashboard.putData("Auto", m_chooser);
    m_chooser.addOption("Red Left", oneCoralAuto(StartingPosition.redLeft));
    m_chooser.addOption("Red Middle", oneCoralAuto(StartingPosition.redMiddle));
    m_chooser.addOption("Red Right", oneCoralAuto(StartingPosition.redRight));
    m_chooser.addOption("Blue Left", oneCoralAuto(StartingPosition.blueLeft));
    m_chooser.addOption("Blue Middle", oneCoralAuto(StartingPosition.blueMiddle));
    m_chooser.addOption("Blue Right", oneCoralAuto(StartingPosition.blueRight));
  }

  // Auto command
  public Command getAutonomousCommand() {
    // return oneCoralAuto();
    return m_chooser.getSelected();
    // return new InstantCommand();
  }
}