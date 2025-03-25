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
            new InstantCommand(() -> clawSubsystem.tiltWristDown()).withTimeout(0.2),
            new ParallelCommandGroup(
                drivetrain.applyRequest(() -> drive.withVelocityX(1.0)).withTimeout(1.5),
                new InstantCommand(() -> elevatorSubsystem.elevatorUp()).withTimeout(1.5)
            ),
            new InstantCommand(() -> clawSubsystem.tiltWristDown()).withTimeout(0.8),
            new InstantCommand(() -> clawSubsystem.outtakeRollers()).withTimeout(0.5),
            drivetrain.applyRequest(() -> idle).withTimeout(0.1),
            new InstantCommand(() -> elevatorSubsystem.elevatorDown()),
            drivetrain.applyRequest(() -> drive.withVelocityX(-1.0)).withTimeout(1.5),
            drivetrain.applyRequest(() -> idle).withTimeout(0.1)
        );
    }
}
