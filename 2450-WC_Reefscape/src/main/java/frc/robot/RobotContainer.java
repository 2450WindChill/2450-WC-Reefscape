// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Camera;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AlignToAprilTagParallel;
import frc.robot.commands.AlignToAprilTagSequential;
import frc.robot.commands.ApproachAprilTag;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SquareToAprilTag;
import frc.robot.commands.StrafeToAprilTag;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.SwerveMode;
import frc.robot.commands.BopAlgae;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralOuttake;
import frc.robot.commands.DeepClimbCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorMovement;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DeepClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.Vector;


import com.pathplanner.lib.commands.PathPlannerAuto;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(SwerveMode.KRAKEN);
  public final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  public final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  // public final DeepClimbSubsystem m_DeepClimbSubsystem = new
  // DeepClimbSubsystem();

  private final XboxController m_driverController = new XboxController(ControllerConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(ControllerConstants.kOperatorControllerPort);

  public final JoystickButton dr_aButton = new JoystickButton(m_driverController, Button.kA.value);
  public final JoystickButton dr_xButton = new JoystickButton(m_driverController, Button.kX.value);
  public final JoystickButton dr_bButton = new JoystickButton(m_driverController, Button.kB.value);


  public final JoystickButton op_aButton = new JoystickButton(m_operatorController, Button.kA.value);
  public final JoystickButton op_bButton = new JoystickButton(m_operatorController, Button.kB.value);

  public final POVButton op_UpDpad = new POVButton(m_operatorController, 180);
  public final POVButton op_DownDpad = new POVButton(m_operatorController, 0);
  public final POVButton op_LeftDpad = new POVButton(m_operatorController, 270);
  public final POVButton op_RightDpad = new POVButton(m_operatorController, 90);

  public final JoystickButton op_xButton = new JoystickButton(m_operatorController, Button.kX.value);
  public final JoystickButton op_yButton = new JoystickButton(m_operatorController, Button.kY.value);

  public final JoystickButton op_leftBumper = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
  public final JoystickButton op_rightBumper = new JoystickButton(m_operatorController, Button.kRightBumper.value);

  public SendableChooser<Command> m_chooser;
  Timer timer = new Timer();
  double time = 0.0;
  private Command Back_Up_Auto;
  private Command One_Meter_Path;
  private Command Algae_Coral_Auto;

  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> (m_driverController.getLeftY()),
            () -> (m_driverController.getLeftX()),
            () -> (m_driverController.getRightX()),
            () -> Constants.isRobotCentric,
            () -> op_leftBumper.getAsBoolean(),
            () -> m_driverController.getPOV()));
    configureControllerBindings();
    configureAutoChooser();
    configureDashboardBindings();
  }

  // TODO: This is where all button mappings go
  private void configureControllerBindings() {
    // Vision commands
    dr_xButton.onTrue(new AlignToAprilTagSequential(m_visionSubsystem, m_drivetrainSubsystem, -0.165, 0.7, Camera.FRONT));
    dr_bButton.onTrue(new AlignToAprilTagSequential(m_visionSubsystem, m_drivetrainSubsystem, 0.165, 0.7, Camera.FRONT));
    dr_aButton.onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro()));

    // Elevator and coral commands
    // op_aButton.whileTrue(new ElevatorMovement(m_coralSubsystem, "down", 0.2));
    // op_yButton.whileTrue(new ElevatorMovement(m_coralSubsystem, "up", 0.2));
    // op_xButton.whileTrue(new CoralIntake(m_coralSubsystem, 0.2));

    op_aButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem, Constants.intakeHeight));
    op_xButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem, Constants.L1Height));
    op_yButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem, Constants.L2Height));
    op_bButton.onTrue(new MoveElevatorToPosition(m_coralSubsystem, Constants.L3Height));
    op_rightBumper.whileTrue(new ElevatorMovement(m_coralSubsystem, "up", 0.2));
    op_leftBumper.whileTrue(new ElevatorMovement(m_coralSubsystem, "down", 0.2));

    m_coralSubsystem.setDefaultCommand(
    new BopAlgae(
    m_coralSubsystem,
    () -> (m_operatorController.getRightTriggerAxis()) * 0.5,
    () -> (m_operatorController.getLeftTriggerAxis()) * 0.5));
  }

  private void configureDashboardBindings() {
    ShuffleboardTab tab = Shuffleboard.getTab("Default");
    tab.add("SquareToAprilTag", new SquareToAprilTag(m_visionSubsystem, m_drivetrainSubsystem, Camera.FRONT)).withWidget(BuiltInWidgets.kCommand);
    tab.add("StrafeToAprilTag", new StrafeToAprilTag(m_visionSubsystem, m_drivetrainSubsystem, 0, Camera.FRONT)).withWidget(BuiltInWidgets.kCommand);
    tab.add("ApproachAprilTag", new ApproachAprilTag(m_visionSubsystem, m_drivetrainSubsystem, 1.3, Camera.FRONT)).withWidget(BuiltInWidgets.kCommand);
    tab.add("AlignToAprilTag", new AlignToAprilTagSequential(m_visionSubsystem, m_drivetrainSubsystem, 0, 1.3, Camera.FRONT)).withWidget(BuiltInWidgets.kCommand);

    tab.add("Intake height", new MoveElevatorToPosition(m_coralSubsystem, -25)).withWidget(BuiltInWidgets.kCommand);
    tab.add("L1 height", new MoveElevatorToPosition(m_coralSubsystem, Constants.L1Height)).withWidget(BuiltInWidgets.kCommand);
    tab.add("L2 height", new MoveElevatorToPosition(m_coralSubsystem, Constants.L2Height)).withWidget(BuiltInWidgets.kCommand);
    tab.add("L3 height", new MoveElevatorToPosition(m_coralSubsystem, Constants.L3Height)).withWidget(BuiltInWidgets.kCommand);
  }

  // Basic auto for testing, backs up after a certain period of time
  private Command autoBackUp() {
    return Commands
        .runOnce(() -> m_drivetrainSubsystem.drive(new Translation2d(2, 0),
            m_drivetrainSubsystem.gyro.getYaw().getValueAsDouble(), true, false))
        .andThen((new WaitCommand(2)))
        .andThen(Commands.runOnce(() -> m_drivetrainSubsystem.drive(new Translation2d(0, 0),
            m_drivetrainSubsystem.gyro.getYaw().getValueAsDouble(), true, false)));
  }

  // Creating different options for auto
  private void configureAutoChooser() {
    // Create auto chooser
    m_chooser = new SendableChooser<>();

    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto Mode", m_chooser);

    // One Meter Pathplanner auto
    One_Meter_Path = new PathPlannerAuto("One_Meter_Auto");

    // TODO: Eventually add all the autos to the chooser
    m_chooser.addOption("One_Meter_Auto", One_Meter_Path);
  }

  // Auto command
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}