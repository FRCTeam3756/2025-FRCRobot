package frc.robot.logic.goals;

public interface GoalHandler {
    void onStart();
    void onUpdate();
    void onStop();

    default boolean areRequirementsMet() {
        return true;
    }

    default boolean isFinished() {
        return false;
    }
}