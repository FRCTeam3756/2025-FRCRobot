// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.*;
import frc.robot.swerve.Drive;

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
    
    Controller.driveTurboButton
        .whileTrue(new InstantCommand(() -> setTurboActive(true)))
        .onFalse(new InstantCommand(() -> setTurboActive(false)));
    Controller.climbButton
        .whileTrue(new InstantCommand(() -> climbSubsystem.climbing(), climbSubsystem))
        .onFalse(new InstantCommand(() -> climbSubsystem.stopClimbing(), climbSubsystem));
    Controller.elevatorUpButton
        .whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorUp()))
        .onFalse(new InstantCommand(() -> elevatorSubsystem.elevatorStop()));
    Controller.elevatorDownButton
        .whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorDown()))
        .onFalse(new InstantCommand(() -> elevatorSubsystem.elevatorStop()));
    Controller.clawTiltUp
        .whileTrue(new InstantCommand(() -> clawSubsystem.tiltWristUp()))
        .onFalse(new InstantCommand(() -> clawSubsystem.tiltWristStop()));
    Controller.clawTiltDown
        .whileTrue(new InstantCommand(() -> clawSubsystem.tiltWristDown()))
        .onFalse(new InstantCommand(() -> clawSubsystem.tiltWristStop()));        
    Controller.clawIntakeButton
        .whileTrue(new InstantCommand(() -> clawSubsystem.intakeGamePiece()))
        .onFalse(new InstantCommand(() -> clawSubsystem.stopRollers()));
    Controller.clawOuttakeButton
        .whileTrue(new InstantCommand(() -> clawSubsystem.outtakeGamePiece()))
        .onFalse(new InstantCommand(() -> clawSubsystem.stopRollers()));
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
