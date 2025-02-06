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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.*;
import frc.robot.constants.*;
import frc.robot.commands.*;

public class RobotContainer {
    private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();
    // private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    // private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CommandXboxController joystick = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private boolean turboActive = false;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MAX_SPEED * (turboActive ? SwerveConstants.TURBO_DRIVE_MULTIPLIER : SwerveConstants.STANDARD_DRIVE_MULTIPLIER)).withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE * ControllerConstants.DEADZONE)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final SwerveSubsystem drivetrain = SwerveConstants.createDrivetrain();
    
    public final Trigger climbUpButton = ControllerConstants.climbUpButton;
    public final Trigger climbDownButton = ControllerConstants.climbDownButton;
    public final Trigger intakeButton = ControllerConstants.intakeButton;
    public final Trigger shootProcessorButton = ControllerConstants.shootProcessorButton;
    public final Trigger shootBargeButton = ControllerConstants.shootBargeButton;
    public final Trigger elevatorUpButton = ControllerConstants.elevatorUpButton;
    public final Trigger elevatorDownButton = ControllerConstants.elevatorDownButton;
    public final Trigger driveTurboButton = ControllerConstants.driveTurboButton;

    public RobotContainer() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * SwerveConstants.MAX_SPEED * (turboActive ? SwerveConstants.TURBO_DRIVE_MULTIPLIER : SwerveConstants.STANDARD_DRIVE_MULTIPLIER)) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * SwerveConstants.MAX_SPEED * (turboActive ? SwerveConstants.TURBO_DRIVE_MULTIPLIER : SwerveConstants.STANDARD_DRIVE_MULTIPLIER)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * SwerveConstants.MAX_ANGULAR_RATE) // Drive counterclockwise with negative X (left)
            )
        );

        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // elevatorUpButton.whileTrue(new ElevatorUp(elevatorSubsystem));
        // elevatorDownButton.whileTrue(new ElevatorDown(elevatorSubsystem));
        driveTurboButton.whileTrue(new InstantCommand(() -> turboActive = true));
        driveTurboButton.onFalse(new InstantCommand(() -> turboActive = false));
        climbUpButton.whileTrue(new ClimbUp(climbSubsystem));
        climbDownButton.whileTrue(new ClimbDown(climbSubsystem));
        // intakeButton.whileTrue(new IntakeAlgae(algaeSubsystem));
        // shootProcessorButton.whileTrue(new ShootProcessor(algaeSubsystem));
        // shootBargeButton.whileTrue(new ShootBarge(algaeSubsystem));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
