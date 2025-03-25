package frc.robot.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class ScoreCoralCenter extends SequentialCommandGroup {
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    public ScoreCoralCenter(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
        addCommands(
            // Drive to the trough and raise elevator
            new ParallelCommandGroup(
                drivetrain.applyRequest(() -> drive.withVelocityX(4.0)).withTimeout(0.5).andThen(drivetrain.applyRequest(() -> idle)),
                new InstantCommand(() -> clawSubsystem.tiltWristDown(), clawSubsystem).withTimeout(0.2).andThen(() -> clawSubsystem.tiltWristStop(), clawSubsystem)
                    .andThen(() -> elevatorSubsystem.elevatorUp(), elevatorSubsystem).withTimeout(1.5).andThen(() -> elevatorSubsystem.elevatorStop(), elevatorSubsystem)
            ),

            // Place the coral
            new InstantCommand(() -> clawSubsystem.tiltWristDown(), clawSubsystem).withTimeout(0.8).andThen(() -> clawSubsystem.tiltWristStop(), clawSubsystem),
            new InstantCommand(() -> clawSubsystem.outtakeRollers(), clawSubsystem).withTimeout(0.5).andThen(() -> clawSubsystem.stopRollers(), clawSubsystem),
            new InstantCommand(() -> clawSubsystem.tiltWristUp(), clawSubsystem).withTimeout(1.0).andThen(() -> clawSubsystem.tiltWristStop(), clawSubsystem),

            // Turn around while driving backwards and lower elevator
            new ParallelCommandGroup(
                drivetrain.applyRequest(() -> drive.withVelocityX(-4.0)).withTimeout(0.5).andThen(drivetrain.applyRequest(() -> idle)),
                new InstantCommand(() -> elevatorSubsystem.elevatorDown(), elevatorSubsystem).withTimeout(1.5).andThen(() -> elevatorSubsystem.elevatorStop(), elevatorSubsystem)
            )
        );
    }
}