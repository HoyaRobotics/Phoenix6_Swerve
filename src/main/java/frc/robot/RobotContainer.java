// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.Pigeon2Subsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();
  private final PoseEstimator poseEstimator = new PoseEstimator(swerveSubsystem, pigeon2Subsystem);

  private static final CommandXboxController driverController = new CommandXboxController(Constants.kDriverControllerPort);
  private static final CommandXboxController operatorController = new CommandXboxController(Constants.kOperatorControllerPort);

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks(
      swerveSubsystem, 
      poseEstimator, 
      () -> driverController.getLeftX(), 
      () -> driverController.getLeftY(),
      () -> driverController.getRightX(), 
      () -> GlobalVariables.fieldRelative, 
      () -> GlobalVariables.maxSpeed));

    configureBindings();
  }

  private void configureBindings() {
    driverController.back().onTrue(new InstantCommand( () -> poseEstimator.setPose(new Pose2d()), poseEstimator));
    driverController.x().onTrue(new InstantCommand( () -> GlobalVariables.fieldRelative = !GlobalVariables.fieldRelative));
    driverController.b().onTrue(new InstantCommand( () -> swerveSubsystem.lock(), swerveSubsystem));
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
