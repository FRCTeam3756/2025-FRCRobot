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
import frc.robot.constants.connection.FieldObjectConstants.Algae;
import frc.robot.constants.connection.FieldObjectConstants.FieldObject;
import frc.robot.constants.connection.FieldObjectConstants.OpponentRobot;
import frc.robot.io.JetsonIO;
import frc.robot.subsystems.mechanical.ClawSubsystem;
import frc.robot.subsystems.mechanical.ClimbingSubsystem;
import frc.robot.subsystems.mechanical.DrivetrainSubsystem;
import frc.robot.subsystems.mechanical.ElevatorSubsystem;

public class VisionSubsystem extends SubsystemBase {

    private final JetsonIO jetson;
    // private final DecisionSubsystem decisionSubsystem;

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ClimbingSubsystem climbingSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private boolean humanDriverControl = false;
    private boolean humanOperatorControl = false;

    private Command activePathCommand;

    public VisionSubsystem(DrivetrainSubsystem drivetrainSubsystem, OdometrySubsystem odometrySubsystem, ClawSubsystem clawSubsystem, ClimbingSubsystem climbingSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.climbingSubsystem = climbingSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.jetson = new JetsonIO(odometrySubsystem);
        // this.decisionSubsystem = new DecisionSubsystem(drivetrainSubsystem, this);
    }

    @Override
    public void periodic() {
        jetson.publishRobotState();
    }

    public void applyDriveCommands(double goalX, double goalY, double goalTheta, double endVelocity) {
        if (humanDriverControl) {
            return;
        }

        Pose2d targetPose = new Pose2d(goalX, goalY, new Rotation2d(goalTheta));

        PathConstraints constraints = new PathConstraints(
                RobotConstants.MAX_AUTONOMOUS_VELOCITY,
                RobotConstants.MAX_AUTONOMOUS_ACCELERATION,
                RobotConstants.MAX_AUTONOMOUS_ANGULAR_VELOCITY,
                RobotConstants.MAX_AUTONOMOUS_ANGULAR_ACCELERATION
        );

        activePathCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                endVelocity
        );

        Pathfinding.setGoalPosition(targetPose.getTranslation());
        Pathfinding.setDynamicObstacles(getDynamicObstacles(), drivetrainSubsystem.getState().Pose.getTranslation());

        activePathCommand.schedule();
    }

    public void applyElevatorCommands(int setpoint) {
        if (humanOperatorControl) {
            return;
        }

        elevatorSubsystem.elevatorToSetpoint(setpoint);
    }

    public void applyClawCommands(boolean intake, boolean outtake) {
        if (humanOperatorControl) {
            return;
        }

        if (intake) {
            clawSubsystem.intakeRollers();
        } else if (outtake) {
            clawSubsystem.outtakeRollers();
        }
    }

    public void applyClimbCommands(boolean climb) {
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
                jetson.getOpponentRobots(drivetrainSubsystem.getState().Pose).stream(),
                jetson.getTeammateRobots(drivetrainSubsystem.getState().Pose).stream()
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
        List<OpponentRobot> opponentRobots = jetson.getOpponentRobots(drivetrainSubsystem.getState().Pose);

        for (int i = 0; i < opponentRobots.size(); i++) {
            double centerX = opponentRobots.get(i).x;
            double centerY = opponentRobots.get(i).y;

            opponentRobotLocations.add(new Translation2d(centerX, centerY));
        }

        return opponentRobotLocations;
    }

    public List<Algae> getAlgae(Pose2d robotPose) {
        return jetson.getAlgae(robotPose);
    }
}
