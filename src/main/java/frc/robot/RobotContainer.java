// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.AutoCommand;
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
  private double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond);
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();
  // private final SendableChooser<Command> autoChooser;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * Controller.DEADZONE)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private enum DriveSpeed {
    SLOW,
    STANDARD,
    TURBO
  }
  
  private DriveSpeed currentDriveSpeed = DriveSpeed.STANDARD;

  public RobotContainer() {
    CameraServer.startAutomaticCapture();

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void setDriverControl() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-Controller.controller.getLeftY() * getCurrentSpeedMultiplier() * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))
            .withVelocityY(-Controller.controller.getLeftX() * getCurrentSpeedMultiplier() * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))
            .withRotationalRate(-Controller.controller.getRightX() * getCurrentSpeedMultiplier() * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))));
    
    Controller.driveTurboButton
        .whileTrue(new InstantCommand(() -> setTurboSpeed()))
        .onFalse(new InstantCommand(() -> setStandardSpeed()));
    Controller.driveSlowButton
        .whileTrue(new InstantCommand(() -> setSlowSpeed()))
        .onFalse(new InstantCommand(() -> setStandardSpeed()));
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
    double speedMultiplier = SwerveConstants.STANDARD_DRIVE_MULTIPLIER;

    switch (currentDriveSpeed) {
      case SLOW:
        speedMultiplier = SwerveConstants.SLOW_DRIVE_MULTIPLIER;
        break;
      case STANDARD:
        speedMultiplier = SwerveConstants.STANDARD_DRIVE_MULTIPLIER;
        break;
      case TURBO:
        speedMultiplier = SwerveConstants.TURBO_DRIVE_MULTIPLIER;
    }

    return speedMultiplier;
  }

  private void setTurboSpeed() {
    currentDriveSpeed = DriveSpeed.TURBO;
  }

  private void setStandardSpeed() {
    currentDriveSpeed = DriveSpeed.STANDARD;
  }

  private void setSlowSpeed() {
    currentDriveSpeed = DriveSpeed.SLOW;
  }

  public Command getAutonomousCommand() {
    return new AutoCommand(drivetrain);
  }
}