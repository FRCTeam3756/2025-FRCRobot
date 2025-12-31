package frc.robot.commands.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.hardware.ChassisConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.JetsonSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

public class DriveToPoseCommand extends Command {

    private final OdometrySubsystem odometrySubsystem;
    private final JetsonSubsystem jetsonSubsystem;
    private final Pose2d targetPose;
    private final double endVelocity;

    private Command pathCommand;

    public DriveToPoseCommand(
            OdometrySubsystem odometrySubsystem,
            DrivetrainSubsystem drivetrainSubsystem,
            JetsonSubsystem jetsonSubsystem,
            Pose2d targetPose,
            double endVelocity
    ) {
        this.odometrySubsystem = odometrySubsystem;
        this.jetsonSubsystem = jetsonSubsystem;
        this.targetPose = targetPose;
        this.endVelocity = endVelocity;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        PathConstraints constraints = new PathConstraints(
                ChassisConstants.MAX_AUTONOMOUS_VELOCITY,
                ChassisConstants.MAX_AUTONOMOUS_ACCELERATION,
                ChassisConstants.MAX_AUTONOMOUS_ANGULAR_VELOCITY,
                ChassisConstants.MAX_AUTONOMOUS_ANGULAR_ACCELERATION
        );

        Pathfinding.setGoalPosition(targetPose.getTranslation());
        Pathfinding.setDynamicObstacles(
                jetsonSubsystem.getDynamicObstacles(),
                odometrySubsystem.getTranslation()
        );

        pathCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                endVelocity
        );

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