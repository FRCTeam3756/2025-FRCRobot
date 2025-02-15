// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.swerve.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

public class RobotContainer {
  // private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  // private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

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
    Controller.climbUpButton.whileTrue(new ClimbUp(climbSubsystem));
    Controller.climbDownButton.whileTrue(new ClimbDown(climbSubsystem));

    // Controller.elevatorUpButton.whileTrue(new ElevatorUp(elevatorSubsystem));
    // Controller.elevatorDownButton.whileTrue(new ElevatorDown(elevatorSubsystem));
    // Controller.intakeButton.whileTrue(new IntakeAlgae(algaeSubsystem));
    // Controller.shootProcessorButton.whileTrue(new ShootProcessor(algaeSubsystem));
    // Controller.shootBargeButton.whileTrue(new ShootBarge(algaeSubsystem));
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
