// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.DriveStraightAuto;
import frc.robot.autonomous.RightDoubleAlgaeAuto;
import frc.robot.constants.connection.ControllerConstants;
import frc.robot.constants.hardware.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.logic.ControlManager;
import frc.robot.logic.GoalManager;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GamepieceSensorSubsystem;
import frc.robot.subsystems.JetsonSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;

public class RobotContainer {

    public static final CommandXboxController controller = new CommandXboxController(ControllerConstants.CONTROLLER_PORT);

    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();

    private final DrivetrainSubsystem drivetrainSubsystem = TunerConstants.createDrivetrain();
    private final OdometrySubsystem odometrySubsystem = new OdometrySubsystem(drivetrainSubsystem);
    private final JetsonSubsystem jetsonSubsystem = new JetsonSubsystem(odometrySubsystem);

    private final ControlManager controlManager = new ControlManager();
    private final GamepieceSensorSubsystem gamepieceSensorSubsystem = new GamepieceSensorSubsystem();
    private final GoalManager goalManager = new GoalManager(controlManager, clawSubsystem, drivetrainSubsystem, odometrySubsystem, jetsonSubsystem, gamepieceSensorSubsystem);

    private enum DriveSpeed {
        SLUG,
        SLOW,
        STANDARD,
        TURBO
    }

    private DriveSpeed currentDriveSpeed = DriveSpeed.STANDARD;
    private final PowerDistributionSubsystem powerDistributionSubsystem = new PowerDistributionSubsystem();

    public RobotContainer() {
        powerDistributionSubsystem.clearStickyFaults();
        powerDistributionSubsystem.setSwitchableChannel(false);
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drivetrainSubsystem.setDefaultCommand(
                drivetrainSubsystem.applyRequest(()
                        -> drivetrainSubsystem.driveRequest.withVelocityX(-controller.getLeftY() * getCurrentSpeedMultiplier() * drivetrainSubsystem.maxSpeed)
                        .withVelocityY(-controller.getLeftX() * getCurrentSpeedMultiplier() * drivetrainSubsystem.maxSpeed)
                        .withRotationalRate(-controller.getRightX() * getCurrentSpeedMultiplier() * drivetrainSubsystem.maxAngularRate)));

        controller.leftStick()
                .whileTrue(new InstantCommand(() -> setTurboSpeed()))
                .onFalse(new InstantCommand(() -> setStandardSpeed()));
        controller.b()
                .whileTrue(new InstantCommand(() -> setSlowSpeed()))
                .onFalse(new InstantCommand(() -> setStandardSpeed()));
        controller.y()
                .whileTrue(new InstantCommand(() -> setSlugSpeed()))
                .onFalse(new InstantCommand(() -> setStandardSpeed()));
        controller.start()
                .whileTrue(new InstantCommand(() -> climbSubsystem.climbing(), climbSubsystem))
                .onFalse(new InstantCommand(() -> climbSubsystem.stopClimbing(), climbSubsystem));
        controller.povUp()
                .whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorUp()))
                .onFalse(new InstantCommand(() -> elevatorSubsystem.elevatorStop()));
        controller.povDown()
                .whileTrue(new InstantCommand(() -> elevatorSubsystem.elevatorDown()))
                .onFalse(new InstantCommand(() -> elevatorSubsystem.elevatorStop()));
        controller.rightBumper()
                .whileTrue(new InstantCommand(() -> clawSubsystem.tiltWristUp()))
                .onFalse(new InstantCommand(() -> clawSubsystem.tiltWristStop()));
        controller.leftBumper()
                .whileTrue(new InstantCommand(() -> clawSubsystem.tiltWristDown()))
                .onFalse(new InstantCommand(() -> clawSubsystem.tiltWristStop()));
        controller.a()
                .whileTrue(new InstantCommand(() -> clawSubsystem.intakeRollers()))
                .onFalse(new InstantCommand(() -> clawSubsystem.stopRollers()));
        controller.x()
                .whileTrue(new InstantCommand(() -> clawSubsystem.outtakeRollers()))
                .onFalse(new InstantCommand(() -> clawSubsystem.stopRollers()));
        controller.back()
                .onTrue(drivetrainSubsystem.runOnce(() -> drivetrainSubsystem.seedFieldCentric()));
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

    public SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Anywhere - Drive Forwards", new DriveStraightAuto(controlManager, drivetrainSubsystem, goalManager));
        autoChooser.addOption("Left Align - Score 2 Algae", new RightDoubleAlgaeAuto(drivetrainSubsystem, goalManager));

        return autoChooser;
    }
}
