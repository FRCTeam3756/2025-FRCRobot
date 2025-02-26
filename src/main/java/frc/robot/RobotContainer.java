// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.swerve.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

public class RobotContainer {
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();

  private final Drive drive = new Drive();
  
  private boolean turboActive = false;

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        Controller.joystickDrive(
            drive,
            () -> -Controller.controller.getLeftY() * getCurrentSpeedMultiplier(),
            () -> -Controller.controller.getLeftX() * getCurrentSpeedMultiplier(),
            () -> -Controller.controller.getRightX() * getCurrentSpeedMultiplier()));

    Controller.controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    
    Controller.driveTurboButton.whileTrue(new InstantCommand(() -> setTurboActive(true)));
    Controller.driveTurboButton.onFalse(new InstantCommand(() -> setTurboActive(false)));
    Controller.climbButton.whileTrue(new InstantCommand(() -> climbSubsystem.climbing()));
    Controller.elevatorUpButton.whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorUp()));
    Controller.elevatorDownButton.whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorDown()));
    Controller.intakeButton.whileTrue(new InstantCommand(() -> clawSubsystem.intakeGamePiece()));
    Controller.shootProcessorButton.whileTrue(new InstantCommand(() -> clawSubsystem.shootProcessor()));
  }

  private double getCurrentSpeedMultiplier() {
    return turboActive ? SwerveConstants.TURBO_DRIVE_MULTIPLIER : SwerveConstants.STANDARD_DRIVE_MULTIPLIER;
  }

  private void setTurboActive(boolean isActive) {
    turboActive = isActive;
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
