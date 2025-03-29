// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Camera;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.CurrentBot;
import frc.robot.Constants.AutoConstants.StartingPosition;
import frc.robot.Constants.AutoConstants.Modifier;
import frc.robot.Constants.AutoConstants.ScoringLevel;
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
import frc.robot.commands.TimedDrive;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DeepClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.lang.invoke.ConstantBootstraps;
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

  public SendableChooser<StartingPosition> m_startingLocationChooser;
  public SendableChooser<ScoringLevel> m_scoreLevelChooser;
  public SendableChooser<Modifier> m_bopHeightChooser;


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
    // dr_bButton.onTrue(Commands
    //     .runOnce(() -> m_drivetrainSubsystem.resetPose(new Pose2d(13.18, 0.5, new Rotation2d(Math.toRadians(-180))))));

    dr_yButton.onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.resetMods()));
    // dr_minusButton.onTrue(new KillDriveCommands(m_drivetrainSubsystem));

    // Only use operator buttons if using the comp robot
    if (currentBotState == CurrentBot.COMP) {
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

      op_UpDpad.whileTrue(new ElevatorMovement(m_coralSubsystem, "up", 0.15));

      op_DownDpad.whileTrue(new ElevatorMovement(m_coralSubsystem, "down", 0.15));
                

      op_LeftDpad.onTrue(Commands.runOnce(() -> m_LEDSubsystem.blinkAllianceColor()))
                .whileTrue(bopLowAlgaeSequence())
                .onFalse(Commands.runOnce(() -> m_LEDSubsystem.setAllianceColor()));

      op_RightDpad.onTrue(Commands.runOnce(() -> m_LEDSubsystem.blinkAllianceColor()))
                  .whileTrue(bopHighAlgaeSequence())
                  .onFalse(Commands.runOnce(() -> m_LEDSubsystem.setAllianceColor()));

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
        new BopAlgae(m_endEffectorSubsystem, m_LEDSubsystem, 0.2));
  }

  private Command bopHighAlgaeSequence() {
    return Commands.parallel(
        new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.highBopAlgae),
        new BopAlgae(m_endEffectorSubsystem, m_LEDSubsystem, 0.2));
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
    tab.add("ON", Commands.runOnce(() -> m_LEDSubsystem.setLEDColor(0, 0, 255, 0)));
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

  private Command autoBuilder(StartingPosition startingPosition, ScoringLevel scoringLevel, Modifier bopHeight) {
    Pose2d scoringPose;
    double scoringHeight;
    Pose2d intakingPose;
    Pose2d secondScoringPose;
    switch (startingPosition) {
      case redLeft:
        m_drivetrainSubsystem.setGyro(180);
        m_drivetrainSubsystem.resetPose(AutoConstants.redLeftStartingPose);
        scoringPose = AutoConstants.redLeftScoringPose;
        intakingPose = Constants.redLeftHumanPlayerStation;
        secondScoringPose = Constants.sevenL;
        break;

      case redMiddle:
        m_drivetrainSubsystem.setGyro(180);
        m_drivetrainSubsystem.resetPose(AutoConstants.redMiddleStartingPose);
        scoringPose = intakingPose = secondScoringPose = AutoConstants.redMiddleScoringPose;
        break;

      case redRight:
        m_drivetrainSubsystem.setGyro(180);
        m_drivetrainSubsystem.resetPose(AutoConstants.redRightStartingPose);
        scoringPose = AutoConstants.redRightScoringPose;
        intakingPose = Constants.redRightHumanPlayerStation;
        secondScoringPose = Constants.sevenL;
      break;

      case blueLeft:
        m_drivetrainSubsystem.setGyro(0);
        m_drivetrainSubsystem.resetPose(AutoConstants.blueLeftStartingPose);
        scoringPose = intakingPose = AutoConstants.blueLeftScoringPose;
        secondScoringPose = Constants.seventeenL;
      break;

      case blueMiddle:
      m_drivetrainSubsystem.setGyro(0);
        m_drivetrainSubsystem.resetPose(AutoConstants.blueMiddleStartingPose);
        scoringPose = intakingPose = secondScoringPose = AutoConstants.blueMiddleScoringPose;
        break;

      case blueRight:
        m_drivetrainSubsystem.setGyro(0);
        m_drivetrainSubsystem.resetPose(AutoConstants.blueRightStartingPose);
        scoringPose = intakingPose = AutoConstants.blueRightScoringPose;
        secondScoringPose = Constants.seventeenL;
        break;

      default:
        scoringPose = intakingPose = secondScoringPose = m_drivetrainSubsystem.getThisPose();
    }

    switch (scoringLevel) {
      case L1:
        scoringHeight = Constants.L1Height;
        break;

      case L2:
        scoringHeight = Constants.L2Height;
        break;

      default:
        scoringHeight = Constants.intakeHeight;
        break;
    }
    
    switch (bopHeight) {
      case ONE_CORAL:
        return Commands.parallel(
          new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.intakeHeight), 
          new CoralIntake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2)
        )
        .andThen(Commands.parallel(
          new MoveToPose(m_drivetrainSubsystem, scoringPose, 5), 
          new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, scoringHeight)
        ))
        .andThen(new CoralOuttake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2));

      case TWO_CORAL:
        return Commands.parallel(
          new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.intakeHeight), 
          new CoralIntake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2)
        )
        .andThen(Commands.parallel(
          new MoveToPose(m_drivetrainSubsystem, scoringPose, 3), 
          new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, scoringHeight)
        ))
        .andThen(new CoralOuttake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2))
        .andThen(
          Commands.parallel(
             new MoveToPose(m_drivetrainSubsystem, intakingPose, 5),
             new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.intakeHeight),
             new CoralIntake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2)
          )
        .andThen(
          Commands.parallel(
            new MoveToPose(m_drivetrainSubsystem, secondScoringPose, 3),
            new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, scoringHeight)
          )
        .andThen(new CoralOuttake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2))
        ));

      case NO_SHOOT:
        return new WaitCommand(7)
        .andThen(Commands.parallel(
            new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.intakeHeight), 
            new CoralIntake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2)
          ))
        .andThen(new MoveToPose(m_drivetrainSubsystem, scoringPose, 5));

      default:
        return Commands.parallel(
          new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, Constants.intakeHeight), 
          new CoralIntake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2)
        )
        .andThen(Commands.parallel(
          new MoveToPose(m_drivetrainSubsystem, scoringPose, 5), 
          new MoveElevatorToPosition(m_coralSubsystem, m_endEffectorSubsystem, m_LEDSubsystem, scoringHeight)
        ))
        .andThen(new CoralOuttake(m_endEffectorSubsystem, m_LEDSubsystem, 0.2));
    }
  }

  private void configureAutoChooser() {
    m_startingLocationChooser = new SendableChooser<>();
    m_scoreLevelChooser = new SendableChooser<>();
    m_bopHeightChooser = new SendableChooser<>();

    // Starting Position Chooser
    SmartDashboard.putData("Starting Position", m_startingLocationChooser);
    m_startingLocationChooser.addOption("Red Left", StartingPosition.redLeft);
    m_startingLocationChooser.addOption("Red Middle", StartingPosition.redMiddle);
    m_startingLocationChooser.addOption("Red Right", StartingPosition.redRight);

    m_startingLocationChooser.addOption("Blue Left", StartingPosition.blueLeft);
    m_startingLocationChooser.addOption("Blue Middle", StartingPosition.blueMiddle);
    m_startingLocationChooser.addOption("Blue Right", StartingPosition.blueRight);

    // Scoring Level Chooser
    SmartDashboard.putData("Scoring Level", m_scoreLevelChooser);
    m_scoreLevelChooser.addOption("L1", ScoringLevel.L1);
    m_scoreLevelChooser.addOption("L2", ScoringLevel.L2);

    // Bop Height Chooser
    SmartDashboard.putData("Bop Height", m_bopHeightChooser);
    m_bopHeightChooser.addOption("One Coral", Modifier.ONE_CORAL);
    m_bopHeightChooser.addOption("No Bop", Modifier.TWO_CORAL);
    m_bopHeightChooser.addOption("No Shoot", Modifier.NO_SHOOT);
  }

  // Auto command
  public Command getAutonomousCommand() {
    return autoBuilder(m_startingLocationChooser.getSelected(), m_scoreLevelChooser.getSelected(), m_bopHeightChooser.getSelected())
    .andThen(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro()));
  }
}