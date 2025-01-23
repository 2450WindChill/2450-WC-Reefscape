// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
  public SendableChooser<Command> m_chooser;

  public RobotContainer() {
     m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> Constants.isRobotCentric,
            m_driverController.leftTrigger()
          )
        );
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

  // Auto command
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
   // Creating different options for auto
  private void configureAutoChooser() {
    m_chooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Mode", m_chooser);
  }
}