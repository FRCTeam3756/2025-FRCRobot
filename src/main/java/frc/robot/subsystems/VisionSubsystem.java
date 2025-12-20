package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.connection.NetworkConstants.JetsonToRio;
import frc.robot.io.JetsonIO;

public class VisionSubsystem extends SubsystemBase {

    private final JetsonIO jetson;

    private final ClawSubsystem clawSubsystem;
    private final ClimbingSubsystem climbingSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private boolean humanDriverControl = false;
    private boolean humanOperatorControl = false;

    private Command activePathCommand;

    public VisionSubsystem(OdometrySubsystem odometrySubsystem, ClawSubsystem clawSubsystem, ClimbingSubsystem climbingSubsystem, ElevatorSubsystem elevatorSubsystem) {
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
                3.0, // max velocity (m/s)
                4.0, // max acceleration (m/s^2)
                Units.degreesToRadians(540), // max angular velocity (rad/s)
                Units.degreesToRadians(720) // max angular acceleration (rad/s^2)
        );

        activePathCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // end velocity
        );

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
}
