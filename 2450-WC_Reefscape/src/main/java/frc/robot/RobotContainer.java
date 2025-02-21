// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AlignToAprilTagParallel;
import frc.robot.commands.AlignToAprilTagSequential;
import frc.robot.commands.ApproachAprilTag;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SquareToAprilTag;
import frc.robot.commands.StrafeToAprilTag;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
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
    m_driverController.x().onTrue(new AlignToAprilTagSequential(m_visionSubsystem, m_drivetrainSubsystem, -0.165, 0.7));
    m_driverController.b().onTrue(new AlignToAprilTagSequential(m_visionSubsystem, m_drivetrainSubsystem, 0.165, 0.7));
    // m_driverController.leftBumper().onTrue(new AlignToAprilTagParallel(m_visionSubsystem, m_drivetrainSubsystem, -0.165, 0.7));
    // m_driverController.rightBumper().onTrue(new AlignToAprilTagParallel(m_visionSubsystem, m_drivetrainSubsystem, 0.165, 0.7));
  }

  private void configureDashboardBindings() {
    ShuffleboardTab tab = Shuffleboard.getTab("Default");
    tab.add("SquareToAprilTag", new SquareToAprilTag(m_visionSubsystem, m_drivetrainSubsystem)).withWidget(BuiltInWidgets.kCommand);
    tab.add("StrafeToAprilTag", new StrafeToAprilTag(m_visionSubsystem, m_drivetrainSubsystem, 0)).withWidget(BuiltInWidgets.kCommand);
    tab.add("ApproachAprilTag", new ApproachAprilTag(m_visionSubsystem, m_drivetrainSubsystem, 1.3)).withWidget(BuiltInWidgets.kCommand);
    tab.add("AlignToAprilTag", new AlignToAprilTagSequential(m_visionSubsystem, m_drivetrainSubsystem, 0, 1.3)).withWidget(BuiltInWidgets.kCommand);
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