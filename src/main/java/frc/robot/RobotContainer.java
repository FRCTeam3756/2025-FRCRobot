// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.swerve.*;

public class RobotContainer {
  // private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  // private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

  private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final Drive drive = new Drive(
    new GyroIOPigeon2(),
    new ModuleIOTalonFX(SwerveConstants.FL_SWERVE_MODULE),
    new ModuleIOTalonFX(SwerveConstants.FR_SWERVE_MODULE),
    new ModuleIOTalonFX(SwerveConstants.BL_SWERVE_MODULE),
    new ModuleIOTalonFX(SwerveConstants.BR_SWERVE_MODULE));
  private final CommandXboxController controller = new CommandXboxController(0);
  private boolean turboActive = false;
  private double currentSpeedMultiplier = (turboActive ? SwerveConstants.TURBO_DRIVE_MULTIPLIER : SwerveConstants.STANDARD_DRIVE_MULTIPLIER);

  
  // private final Trigger intakeButton = ControllerConstants.intakeButton;
  // private final Trigger shootProcessorButton = ControllerConstants.shootProcessorButton;
  // private final Trigger shootBargeButton = ControllerConstants.shootBargeButton;
  private final Trigger climbUpButton = ControllerConstants.climbUpButton;
  private final Trigger climbDownButton = ControllerConstants.climbDownButton;
  private final Trigger elevatorUpButton = ControllerConstants.elevatorUpButton;
  private final Trigger elevatorDownButton = ControllerConstants.elevatorDownButton;
  private final Trigger driveTurboButton = ControllerConstants.driveTurboButton;
  // private final Trigger resetGyroButton = ControllerConstants.resetGyroButton;

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * currentSpeedMultiplier,
            () -> -controller.getLeftX() * currentSpeedMultiplier,
            () -> -controller.getRightX() * currentSpeedMultiplier));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    
    driveTurboButton.whileTrue(new InstantCommand(() -> turboActive = true));
    driveTurboButton.onFalse(new InstantCommand(() -> turboActive = false));
    climbUpButton.whileTrue(new ClimbUp(climbSubsystem));
    climbDownButton.whileTrue(new ClimbDown(climbSubsystem));

    elevatorUpButton.whileTrue(new ElevatorUp(elevatorSubsystem));
    elevatorDownButton.whileTrue(new ElevatorDown(elevatorSubsystem));
    // intakeButton.whileTrue(new IntakeAlgae(algaeSubsystem));
    // shootProcessorButton.whileTrue(new ShootProcessor(algaeSubsystem));
    // shootBargeButton.whileTrue(new ShootBarge(algaeSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
