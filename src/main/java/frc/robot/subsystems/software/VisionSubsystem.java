package frc.robot.subsystems.software;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RulebookConstants;
import frc.robot.constants.connection.FieldObjectConstants.FieldObject;
import frc.robot.constants.connection.FieldObjectConstants.OpponentRobot;
import frc.robot.constants.connection.NetworkConstants.JetsonToRio;
import frc.robot.io.JetsonIO;
import frc.robot.subsystems.mechanical.ClawSubsystem;
import frc.robot.subsystems.mechanical.ClimbingSubsystem;
import frc.robot.subsystems.mechanical.DrivetrainSubsystem;
import frc.robot.subsystems.mechanical.ElevatorSubsystem;

public class VisionSubsystem extends SubsystemBase {

    private final JetsonIO jetson;

    private final DrivetrainSubsystem drivetrain;
    private final ClawSubsystem clawSubsystem;
    private final ClimbingSubsystem climbingSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private boolean humanDriverControl = false;
    private boolean humanOperatorControl = false;

    private Command activePathCommand;

    public VisionSubsystem(DrivetrainSubsystem drivetrain, OdometrySubsystem odometrySubsystem, ClawSubsystem clawSubsystem, ClimbingSubsystem climbingSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.drivetrain = drivetrain;
        this.clawSubsystem = clawSubsystem;
        this.climbingSubsystem = climbingSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.jetson = new JetsonIO(odometrySubsystem);
    }

    @Override
    public void periodic() {
        jetson.publishRobotState();

        applyDriveCommands(
                jetson.getDouble(JetsonToRio.DESIRED_X),
                jetson.getDouble(JetsonToRio.DESIRED_Y),
                jetson.getDouble(JetsonToRio.DESIRED_THETA)
        );

        applyElevatorCommands(
                jetson.getInteger(JetsonToRio.DESIRED_ELEVATOR_SETPOINT)
        );

        applyClawCommands(
                jetson.getBoolean(JetsonToRio.DESIRED_INTAKE),
                jetson.getBoolean(JetsonToRio.DESIRED_OUTTAKE)
        );

        applyClimbCommands(
                jetson.getBoolean(JetsonToRio.DESIRED_CLIMB)
        );
    }

    private void applyDriveCommands(double x, double y, double theta) {
        if (humanDriverControl) {
            return;
        }

        Pose2d targetPose = new Pose2d(x, y, new Rotation2d(theta));

        PathConstraints constraints = new PathConstraints(
                RobotConstants.MAX_AUTONOMOUS_VELOCITY,
                RobotConstants.MAX_AUTONOMOUS_ACCELERATION,
                RobotConstants.MAX_AUTONOMOUS_ANGULAR_VELOCITY,
                RobotConstants.MAX_AUTONOMOUS_ANGULAR_ACCELERATION
        );

        activePathCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // end velocity
        );

        Pathfinding.setGoalPosition(targetPose.getTranslation());
        Pathfinding.setDynamicObstacles(getDynamicObstacles(), drivetrain.getState().Pose.getTranslation());

        activePathCommand.schedule();
    }

    private void applyElevatorCommands(int setpoint) {
        if (humanOperatorControl) {
            return;
        }

        elevatorSubsystem.elevatorToSetpoint(setpoint);
    }

    private void applyClawCommands(boolean intake, boolean outtake) {
        if (humanOperatorControl) {
            return;
        }

        if (intake) {
            clawSubsystem.intakeRollers();
        } else if (outtake) {
            clawSubsystem.outtakeRollers();
        }
    }

    private void applyClimbCommands(boolean climb) {
        if (humanOperatorControl) {
            return;
        }

        if (climb) {
            climbingSubsystem.climbing();
        }
    }

    public void forceDrivingControl(boolean control) {
        humanDriverControl = control;
    }

    public void forceOperatorControl(boolean control) {
        humanOperatorControl = control;
    }

    public boolean isHumanDriverControl() {
        return humanDriverControl;
    }

    public boolean isHumanOperatorControl() {
        return humanOperatorControl;
    }

    public List<Pair<Translation2d, Translation2d>> getDynamicObstacles() {
        List<Pair<Translation2d, Translation2d>> obstacleLocations = new ArrayList<>();

        List<FieldObject> obstacles = Stream.concat(
                jetson.getOpponentRobots().stream(),
                jetson.getTeammateRobots().stream()
        ).toList();

        for (int i = 0; i < obstacles.size(); i++) {
            double centerX = obstacles.get(i).x;
            double centerY = obstacles.get(i).y;

            Translation2d minCorner = new Translation2d(
                    centerX - RulebookConstants.MAX_ROBOT_WIDTH / 2.0,
                    centerY - RulebookConstants.MAX_ROBOT_LENGTH / 2.0
            );

            Translation2d maxCorner = new Translation2d(
                    centerX + RulebookConstants.MAX_ROBOT_WIDTH / 2.0,
                    centerY + RulebookConstants.MAX_ROBOT_LENGTH / 2.0
            );

            obstacleLocations.add(new Pair<>(minCorner, maxCorner));
        }

        return obstacleLocations;
    }

    public List<Translation2d> getOpponentRobotLocations() {
        List<Translation2d> opponentRobotLocations = new ArrayList<>();

        List<OpponentRobot> opponentRobots = jetson.getOpponentRobots();

        for (int i = 0; i < opponentRobots.size(); i++) {
            double centerX = opponentRobots.get(i).x;
            double centerY = opponentRobots.get(i).y;

            opponentRobotLocations.add(new Translation2d(centerX, centerY));
        }

        return opponentRobotLocations;
    }

    public List<Translation2d> getAlgaeLocations() {
        List<Translation2d> algaeLocations = new ArrayList<>();

        return algaeLocations;
    }
}
