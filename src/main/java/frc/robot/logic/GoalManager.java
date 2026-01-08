package frc.robot.logic;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.logic.goals.FindAlgaeGoal;
import frc.robot.logic.goals.GoalHandler;
import frc.robot.logic.goals.IdleGoal;
import frc.robot.logic.goals.LineDefenseGoal;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamepieceSensorSubsystem;
import frc.robot.subsystems.JetsonSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

public class GoalManager extends SubsystemBase {

    public enum Goal {
        IDLE,
        SCORE_CORAL,
        SCORE_ALGAE,
        FIND_CORAL,
        FIND_ALGAE,
        LINE_DEFENSE,
        FOLLOW_HUMAN
    }

    private Goal currentGoal = Goal.IDLE;
    private Goal previousGoal = Goal.IDLE;

    private GoalHandler currentHandler;
    private GoalHandler previousHandler;

    private final ControlManager controlManager;
    private final ClawSubsystem clawSubsystem;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final OdometrySubsystem odometrySubsystem;
    private final JetsonSubsystem jetsonSubsystem;
    private final GamepieceSensorSubsystem gamepieceSensorSubsystem;

    public GoalManager(ControlManager controlManager, ClawSubsystem clawSubsystem, DrivetrainSubsystem drivetrainSubsystem, OdometrySubsystem odometrySubsystem, JetsonSubsystem jetsonSubsystem, GamepieceSensorSubsystem gamepieceSensorSubsystem) {
        this.controlManager = controlManager;
        this.clawSubsystem = clawSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.jetsonSubsystem = jetsonSubsystem;
        this.gamepieceSensorSubsystem = gamepieceSensorSubsystem;
    }

    public void runGoal(Goal goal) {
        previousGoal = currentGoal;
        currentGoal = goal;

        if (currentGoal != previousGoal) {
            if (previousHandler != null) {
                previousHandler.onStop();
            }

            currentHandler = switch (currentGoal) {
                case IDLE ->
                    new IdleGoal(controlManager);
                case SCORE_CORAL -> null;
                case SCORE_ALGAE -> null;
                case FIND_CORAL -> null;
                case FIND_ALGAE -> 
                    new FindAlgaeGoal(controlManager, clawSubsystem, drivetrainSubsystem, odometrySubsystem, jetsonSubsystem, gamepieceSensorSubsystem);
                case LINE_DEFENSE ->
                    new LineDefenseGoal(controlManager, drivetrainSubsystem, odometrySubsystem, jetsonSubsystem);
                case FOLLOW_HUMAN -> null;
            };

            if (currentHandler != null) {
                currentHandler.onStart();
            }
        }

        if (currentHandler != null) {
            if (!currentHandler.areRequirementsMet()) {
                currentHandler.onStop();
            } else if (currentHandler.isFinished()) {
                currentHandler.onStop();
            } else {
                currentHandler.onUpdate();
            }
        }

        previousGoal = currentGoal;
        previousHandler = currentHandler;
    }
}
