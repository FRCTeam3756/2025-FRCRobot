// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.subsystems.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.io.ControllerIO;
import frc.robot.io.JetsonIO;
import frc.robot.io.PowerDistributionIO;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

public class RobotContainer {

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
    private final double MaxAngularRate = Units.RotationsPerSecond.of(1.0).in(Units.RadiansPerSecond);

    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final OdometrySubsystem odometry = new OdometrySubsystem(drivetrain);
    public final JetsonIO jetson = new JetsonIO(odometry);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * ControllerIO.DEADZONE).withRotationalDeadband(MaxAngularRate * ControllerIO.DEADZONE)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private enum DriveSpeed {
        SLUG,
        SLOW,
        STANDARD,
        TURBO
    }

    private DriveSpeed currentDriveSpeed = DriveSpeed.STANDARD;
    private final PowerDistributionIO powerDistributionHub = new PowerDistributionIO();

    public RobotContainer() {
        powerDistributionHub.clearStickyFaults();
        powerDistributionHub.setSwitchableChannel(false);
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(()
                        -> drive.withVelocityX(-ControllerIO.controller.getLeftY() * getCurrentSpeedMultiplier() * MaxSpeed)
                        .withVelocityY(-ControllerIO.controller.getLeftX() * getCurrentSpeedMultiplier() * MaxSpeed)
                        .withRotationalRate(-ControllerIO.controller.getRightX() * getCurrentSpeedMultiplier() * MaxAngularRate)));

        ControllerIO.driveTurboButton
                .whileTrue(new InstantCommand(() -> setTurboSpeed()))
                .onFalse(new InstantCommand(() -> setStandardSpeed()));
        ControllerIO.driveSlowButton
                .whileTrue(new InstantCommand(() -> setSlowSpeed()))
                .onFalse(new InstantCommand(() -> setStandardSpeed()));
        ControllerIO.driveSlugButton
                .whileTrue(new InstantCommand(() -> setSlugSpeed()))
                .onFalse(new InstantCommand(() -> setStandardSpeed()));
        ControllerIO.climbButton
                .whileTrue(new InstantCommand(() -> climbSubsystem.climbing(), climbSubsystem))
                .onFalse(new InstantCommand(() -> climbSubsystem.stopClimbing(), climbSubsystem));
        ControllerIO.elevatorUpButton
                .whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorUp()))
                .onFalse(new InstantCommand(() -> elevatorSubsystem.elevatorStop()));
        ControllerIO.elevatorDownButton
                .whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorDown()))
                .onFalse(new InstantCommand(() -> elevatorSubsystem.elevatorStop()));
        ControllerIO.clawTiltUp
                .whileTrue(new InstantCommand(() -> clawSubsystem.tiltWristUp()))
                .onFalse(new InstantCommand(() -> clawSubsystem.tiltWristStop()));
        ControllerIO.clawTiltDown
                .whileTrue(new InstantCommand(() -> clawSubsystem.tiltWristDown()))
                .onFalse(new InstantCommand(() -> clawSubsystem.tiltWristStop()));
        ControllerIO.clawIntakeButton
                .whileTrue(new InstantCommand(() -> clawSubsystem.intakeRollers()))
                .onFalse(new InstantCommand(() -> clawSubsystem.stopRollers()));
        ControllerIO.clawOuttakeButton
                .whileTrue(new InstantCommand(() -> clawSubsystem.outtakeRollers()))
                .onFalse(new InstantCommand(() -> clawSubsystem.stopRollers()));
        ControllerIO.resetGyroScope
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private double getCurrentSpeedMultiplier() {
        double speedMultiplier = DriveConstants.STANDARD_DRIVE_MULTIPLIER;

        switch (currentDriveSpeed) {
            case SLOW ->
                speedMultiplier = DriveConstants.SLOW_DRIVE_MULTIPLIER;
            case STANDARD ->
                speedMultiplier = DriveConstants.STANDARD_DRIVE_MULTIPLIER;
            case TURBO ->
                speedMultiplier = DriveConstants.TURBO_DRIVE_MULTIPLIER;
            case SLUG ->
                speedMultiplier = DriveConstants.SLUG_DRIVE_MULTIPLIER;
        }

        return speedMultiplier;
    }

    private void setSlugSpeed() {
        currentDriveSpeed = DriveSpeed.SLUG;
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

    public SendableChooser<String> buildAutoChooser() {
        SendableChooser<String> autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Anywhere - Drive Forwards", "DriveStraightAuto");
        autoChooser.addOption("Middle Align - Grab Algae", "MiddleReefAlgaeAuto");
        autoChooser.addOption("Middle Align - Score Coral", "MiddleScoreCoralAuto");
        autoChooser.addOption("Left Align - Score Coral", "LeftScoreCoralAuto");
        autoChooser.addOption("Left Align - Score Coral and To Reef", "LeftScoreCoralAndToReefAuto");
        autoChooser.addOption("Left Align - Score 2 Algae", "LeftDoubleAlgaeAuto");
        autoChooser.addOption("Left Align - Push Teammate", "LeftPushTeammateAuto");
        autoChooser.addOption("Right Align - Score Coral", "RightScoreCoralAuto");
        autoChooser.addOption("Right Align - Score 2 Algae", "RightDoubleAlgaeAuto");
        autoChooser.addOption("Right Align - Push Teammate", "RightPushTeammateAuto");
        autoChooser.addOption("Right Align - Score Coral and To Reef", "RightScoreCoralAndToReefAuto");

        return autoChooser;
    }
}
