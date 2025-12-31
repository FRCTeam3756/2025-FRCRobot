package frc.robot.commands.drivetrain;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.hardware.ChassisConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowWaypointPathCommand extends Command {

    private final List<Pose2d> poses;
    private final double endHeading;
    private final double endVelocity;

    private Command pathCommand;

    public FollowWaypointPathCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            List<Pose2d> poses,
            double endHeading,
            double endVelocity
    ) {
        this.poses = poses;
        this.endHeading = endHeading;
        this.endVelocity = endVelocity;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        if (poses == null || poses.isEmpty()) {
            return;
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathConstraints constraints = new PathConstraints(
                ChassisConstants.MAX_AUTONOMOUS_VELOCITY,
                ChassisConstants.MAX_AUTONOMOUS_ACCELERATION,
                ChassisConstants.MAX_AUTONOMOUS_ANGULAR_VELOCITY,
                ChassisConstants.MAX_AUTONOMOUS_ANGULAR_ACCELERATION
        );

        LinearVelocity velocity
                = LinearVelocity.ofBaseUnits(endVelocity, LinearVelocityUnit.combine(Units.Meters, Units.Second));

        GoalEndState goalEndState = new GoalEndState(velocity, new Rotation2d(endHeading));

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                goalEndState
        );

        pathCommand = AutoBuilder.followPath(path);
        pathCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.cancel();
        }
    }
}
