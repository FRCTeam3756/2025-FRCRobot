// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.subsystems.*;

public class OldRobotContainer {
    // private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    // private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

    private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CommandXboxController joystick = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private boolean turboActive = false;
    private double currentSpeedMultiplier = (turboActive ? SwerveConstants.TURBO_DRIVE_MULTIPLIER : SwerveConstants.STANDARD_DRIVE_MULTIPLIER);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(calculateDriveVelocity(ControllerConstants.DEADZONE))
            .withRotationalDeadband(calculateAngularVelocity(ControllerConstants.DEADZONE))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final OldSwerveSubsystem drivetrain = SwerveConstants.createDrivetrain();
    
    // private final Trigger intakeButton = ControllerConstants.intakeButton;
    // private final Trigger shootProcessorButton = ControllerConstants.shootProcessorButton;
    // private final Trigger shootBargeButton = ControllerConstants.shootBargeButton;
    private final Trigger climbUpButton = ControllerConstants.climbUpButton;
    private final Trigger climbDownButton = ControllerConstants.climbDownButton;
    private final Trigger elevatorUpButton = ControllerConstants.elevatorUpButton;
    private final Trigger elevatorDownButton = ControllerConstants.elevatorDownButton;
    private final Trigger driveTurboButton = ControllerConstants.driveTurboButton;
    private final Trigger resetGyroButton = ControllerConstants.resetGyroButton;

    public OldRobotContainer() {
        configureDrivetrain();
        configureButtonBindings();
    }

    private void configureDrivetrain() {
        drivetrain.seedFieldCentric();
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(calculateDriveVelocity(joystick.getLeftY()))
                    .withVelocityY(calculateDriveVelocity(-joystick.getLeftX()))
                    .withRotationalRate(calculateAngularVelocity(-joystick.getRightX()))
            )
        );
    }

    private double calculateDriveVelocity(double input) {
        return input * SwerveConstants.MAX_SPEED * currentSpeedMultiplier;
    }

    private double calculateAngularVelocity(double input) {
        return input * SwerveConstants.MAX_ANGULAR_RATE;
    }

    private void configureButtonBindings() {
        resetGyroButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
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
        return Commands.print("No autonomous command configured");
    }
}
