package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.FollowWaypointPathCommand;
import frc.robot.logic.GoalManager;
import frc.robot.logic.GoalManager.Goal;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RightDoubleAlgaeAuto extends SequentialCommandGroup implements RamFernoAutoInterface {

    public RightDoubleAlgaeAuto(
            DrivetrainSubsystem drivetrainSubsystem,
            GoalManager goalManager
    ) {
        Pose2d path2waypoint1 = new Pose2d(
                new Translation2d(6, 1.5),
                new Rotation2d(-Math.PI / 2)
        );
        Pose2d path2endPoint = new Pose2d(
                new Translation2d(6, 1.5),
                new Rotation2d(-Math.PI / 2)
        );
        List<Pose2d> path2poses = List.of(
                path2waypoint1,
                path2endPoint
        );

        addCommands(
                followPath("RightDoubleAlgaePath1"),
                new InstantCommand(()
                        -> goalManager.runGoal(Goal.FIND_ALGAE)
                ),
                new FollowWaypointPathCommand(
                        drivetrainSubsystem,
                        path2poses,
                        -(Math.PI / 2),
                        0
                )
        );
    }
}
