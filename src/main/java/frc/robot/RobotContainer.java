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
  // private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();

  private final Drive drive = new Drive();
  
  private int elevatorHeight = 1;
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
    Controller.climbUpButton.whileTrue(new InstantCommand(() -> climbSubsystem.climbingUp()));
    Controller.climbDownButton.whileTrue(new InstantCommand(() -> climbSubsystem.climbingDown()));
    Controller.elevatorUpButton.whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorUp(elevatorHeight)));
    Controller.elevatorDownButton.whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorDown(elevatorHeight)));
    // Controller.intakeButton.whileTrue(new IntakeAlgae(clawSubsystem));
    // Controller.shootProcessorButton.whileTrue(new ShootProcessor(clawSubsystem));
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
