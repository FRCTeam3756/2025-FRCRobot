package frc.robot.subsystems.software;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autonomous.goals.GoalHandler;
import frc.robot.autonomous.goals.IdleGoal;
import frc.robot.autonomous.goals.LineDefenseGoal;
import frc.robot.subsystems.mechanical.DrivetrainSubsystem;

public class DecisionSubsystem extends SubsystemBase {

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

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final VisionSubsystem visionSubsystem;

    public DecisionSubsystem(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void periodic() {
        if (currentGoal != previousGoal) {
            if (previousHandler != null) {
                previousHandler.onStop();
            }

            currentHandler = switch (currentGoal) {
                case IDLE ->
                    new IdleGoal(drivetrainSubsystem, visionSubsystem);
                case SCORE_CORAL -> null;
                case SCORE_ALGAE -> null;
                case FIND_CORAL -> null;
                case FIND_ALGAE -> null;
                case LINE_DEFENSE ->
                    new LineDefenseGoal(drivetrainSubsystem, visionSubsystem);
                case FOLLOW_HUMAN -> null;
            };

            if (currentHandler != null) {
                currentHandler.onStart();
            }
        }

        if (currentHandler != null) {
            if (currentHandler.areRequirementsMet()) {
                currentHandler.onUpdate();
            } else {
                currentHandler.onStop();
            }
        }

        previousGoal = currentGoal;
        previousHandler = currentHandler;
    }

    public void setGoal(Goal goal) {
        currentGoal = goal;
    }
}
