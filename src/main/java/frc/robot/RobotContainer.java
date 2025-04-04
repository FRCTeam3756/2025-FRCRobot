// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
  private double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond);

  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * Controller.DEADZONE).withRotationalDeadband(MaxAngularRate * Controller.DEADZONE)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private enum DriveSpeed {
    SLOW,
    STANDARD,
    TURBO,
    SLUG
  }
  
  private DriveSpeed currentDriveSpeed = DriveSpeed.STANDARD;

  public RobotContainer() {
    setupCamera();
    registerNamedCommands();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-Controller.controller.getLeftY() * getCurrentSpeedMultiplier() * TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond))
            .withVelocityY(-Controller.controller.getLeftX() * getCurrentSpeedMultiplier() * TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond))
            .withRotationalRate(-Controller.controller.getRightX() * getCurrentSpeedMultiplier() * TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond))));
    
    Controller.driveTurboButton
        .whileTrue(new InstantCommand(() -> setTurboSpeed()))
        .onFalse(new InstantCommand(() -> setStandardSpeed()));
    Controller.driveSlowButton
    
        .whileTrue(new InstantCommand(() -> setSlowSpeed()))
        .onFalse(new InstantCommand(() -> setStandardSpeed()));
    Controller.driveSlugButton
        .whileTrue(new InstantCommand(() -> setSlugSpeed()))
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
        .whileTrue(new InstantCommand(() -> clawSubsystem.intakeRollers()))
        .onFalse(new InstantCommand(() -> clawSubsystem.stopRollers()));
    Controller.clawOuttakeButton
        .whileTrue(new InstantCommand(() -> clawSubsystem.outtakeRollers()))
        .onFalse(new InstantCommand(() -> clawSubsystem.stopRollers()));
    Controller.resetGyroScope
        .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
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
        break;
      case SLUG:
        speedMultiplier = SwerveConstants.SLUG_DRIVE_MULTIPLIER;
        break;
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

  private void setSlugSpeed() {
    currentDriveSpeed = DriveSpeed.SLUG;
  }

  public SendableChooser<String> buildAutoChooser() {
    SendableChooser<String> autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("Anywhere - Drive Forwards", "DriveStraightAuto");
    autoChooser.addOption("Middle Align - Grab Algae", "MiddleReefAlgaeAuto");
    autoChooser.addOption("Middle Align - Score Coral", "MiddleScoreCoralAuto");
    autoChooser.addOption("Left Align - Score Coral", "LeftScoreCoralAuto");
    autoChooser.addOption("Left Align - Score 2 Algae", "LeftDoubleAlgaeAuto");
    autoChooser.addOption("Left Align - Push Teammate", "LeftPushTeammateAuto");
    autoChooser.addOption("Right Align - Score Coral", "RightScoreCoralAuto");
    autoChooser.addOption("Right Align - Score 2 Algae", "RightDoubleAlgaeAuto");
    autoChooser.addOption("Right Align - Push Teammate", "RightPushTeammateAuto");
    autoChooser.addOption("Right Align - Score Coral and Through Reef", "RightScoreCoralAndThroughReefAuto");

    return autoChooser;
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("intake", new ClawCommand(clawSubsystem, 1, 0.30, true));
    NamedCommands.registerCommand("outtake", new ClawCommand(clawSubsystem, 3, 0.20, false));

    NamedCommands.registerCommand("clawAwayFromElevator", new WristCommand(clawSubsystem, -0.08, 0.3, 0.0));
    NamedCommands.registerCommand("clawFromTopToTrough", new WristCommand(clawSubsystem, -0.08, 0.8, 0.0));
    NamedCommands.registerCommand("clawFromTopToCoralAlgae", new WristCommand(clawSubsystem, -0.08, 1.0, 0.0));
    NamedCommands.registerCommand("clawFromTopToReefAlgae", new WristCommand(clawSubsystem, -0.15, 1.2, 2.0));
    NamedCommands.registerCommand("clawFromTopToProcessor", new WristCommand(clawSubsystem, -0.08, 0.8, 0.0));

    NamedCommands.registerCommand("elevatorFromBaseToTrough", new ElevatorCommand(elevatorSubsystem, 0.5, 1));
    NamedCommands.registerCommand("elevatorFromBaseToBottomAlgae", new ElevatorCommand(elevatorSubsystem, 0.5, 4));
    NamedCommands.registerCommand("elevatorFromBaseToTopAlgae", new ElevatorCommand(elevatorSubsystem, 0.5, 5));
  }

  private void setupCamera() {
    UsbCamera camera = CameraServer.startAutomaticCapture();

    camera.setResolution(640, 480);
    camera.setFPS(30);

    CameraServer.startAutomaticCapture(camera);
  }

  public void enableSwitchableChannel(boolean enable) {
    try (PowerDistribution powerDistributionHub = new PowerDistribution()) {
      powerDistributionHub.setSwitchableChannel(enable);
    } catch (Exception e) {
      System.out.println("PDH Error:" + e);
    }
  }
}