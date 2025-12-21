package frc.robot.subsystems.software;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autonomous.goals.GoalHandler;
import frc.robot.autonomous.goals.LineDefenseGoal;
import frc.robot.subsystems.mechanical.DrivetrainSubsystem;

public class DecisionSubsystem extends SubsystemBase {

    public enum Goal {
        IDLE,
        SCORE_CORAL,
        SCORE_ALGAE,
        FIND_CORAL,
        FIND_ALGAE,
        LINE_DEFENSE
    }

    private Goal currentGoal = Goal.IDLE;
    private Goal previousGoal = Goal.IDLE;

    private final Map<Goal, GoalHandler> handlers;

    public DecisionSubsystem(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        handlers = Map.of(
            Goal.LINE_DEFENSE, new LineDefenseGoal(drivetrainSubsystem, visionSubsystem)
        );
    }

    @Override
    public void periodic() {
        if (currentGoal != previousGoal) {
            if (previousGoal != null) handlers.get(previousGoal).onStop();
            if (currentGoal != null) handlers.get(currentGoal).onStart();
        }

        GoalHandler handler = handlers.get(currentGoal);
        if (handler.areRequirementsMet()) {
            handler.onUpdate();
        } else {
            handler.onStop();
        }
        
        previousGoal = currentGoal;
    }

    public void setGoal(Goal goal) {
        currentGoal = goal;
    }
}
