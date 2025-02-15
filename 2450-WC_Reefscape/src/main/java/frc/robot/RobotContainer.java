// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveMode;
import frc.robot.commands.BopAlgae;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralOuttake;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorMovement;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

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
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(SwerveMode.NEO);
  //public final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

  private final XboxController m_driverController = new XboxController(ControllerConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(ControllerConstants.kOperatorControllerPort);

  
  public final JoystickButton op_aButton = new JoystickButton(m_operatorController, Button.kA.value);
  public final JoystickButton op_bButton = new JoystickButton(m_operatorController, Button.kB.value);

  public final POVButton op_UpDpad = new POVButton(m_operatorController, 0);
  public final POVButton op_DownDpad = new POVButton(m_operatorController, 180);

  public final JoystickButton op_xButton = new JoystickButton(m_operatorController, Button.kX.value);
  public final JoystickButton op_yButton = new JoystickButton(m_operatorController, Button.kY.value);

  public final JoystickButton op_leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);



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
            () -> (m_driverController.getLeftY() * 0.3),
            () -> (m_driverController.getLeftX() * 0.3),
            () -> (m_driverController.getRightX() * 0.3),
            () -> Constants.isRobotCentric,
            () -> op_leftBumper.getAsBoolean(),
            () -> m_driverController.getPOV()));
    configureControllerBindings();
    configureAutoChooser();
    configureDashboardBindings();
 }

  // TODO: This is where all button mappings go
  private void configureControllerBindings() {

    // op_DownDpad.onTrue(new ElevatorMovement(m_coralSubsystem, "down", 0.2));
    // op_UpDpad.onTrue(new ElevatorMovement(m_coralSubsystem, "up", 0.2));

    // m_coralSubsystem.setDefaultCommand(
    //   new BopAlgae(
    //       m_coralSubsystem,
    //       () -> m_operatorController.getRightTriggerAxis(),
    //       () -> m_operatorController.getLeftTriggerAxis()));
  }

  private void configureDashboardBindings() {
    ShuffleboardTab tab = Shuffleboard.getTab("Default");

    // tab.add("0", new MoveElevatorToPosition(m_coralSubsystem, 0)).withWidget(BuiltInWidgets.kCommand);
    // tab.add("10", new MoveElevatorToPosition(m_coralSubsystem, 10)).withWidget(BuiltInWidgets.kCommand);
    // tab.add("20", new MoveElevatorToPosition(m_coralSubsystem, 20)).withWidget(BuiltInWidgets.kCommand);
    // tab.add("50", new MoveElevatorToPosition(m_coralSubsystem, 50)).withWidget(BuiltInWidgets.kCommand);
    // tab.add("200", new MoveElevatorToPosition(m_coralSubsystem, 200)).withWidget(BuiltInWidgets.kCommand);
    // tab.add("500", new MoveElevatorToPosition(m_coralSubsystem, 500)).withWidget(BuiltInWidgets.kCommand);
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
    m_chooser.addOption("One_Meter_Auto", Back_Up_Auto);
  }

  // Auto command
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}