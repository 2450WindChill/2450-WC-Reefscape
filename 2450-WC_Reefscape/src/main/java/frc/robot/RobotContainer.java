// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveMode;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(SwerveMode.KRAKEN);
  private final CommandXboxController m_driverController = new CommandXboxController(
      ControllerConstants.kDriverControllerPort);
  public SendableChooser<Command> m_chooser;
  Timer timer = new Timer();
  double time = 0.0;
  private Command Back_Up_Auto;
  private Command Algae_Coral_Auto;

  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> Constants.isRobotCentric,
            m_driverController.leftTrigger()));
    configureControllerBindings();
    configureAutoChooser();
    configureDashboardBindings();
  }

  // TODO: This is where all button mappings go
  private void configureControllerBindings() {
  }

  private void configureDashboardBindings() {
    ShuffleboardTab tab = Shuffleboard.getTab("Default");

    // Example tab:
    // tab.add("Title", new SquareToAprilTag(m_poseEstimator, m_drivetrainSubsystem)).withWidget(BuiltInWidgets.kCommand);
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

    // Set the back up auto to the autoBackUp function
    Back_Up_Auto = autoBackUp();

    // TODO: Eventually set up pathplanner to add paths along with the scoring
    // Algae_Coral_Auto = new PathPlannerAuto("Test_Auto");

    // TODO: Eventually add all the autos to the chooser
    m_chooser.addOption("Back_Up_Auto", Back_Up_Auto);
  }

  // Auto command
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}