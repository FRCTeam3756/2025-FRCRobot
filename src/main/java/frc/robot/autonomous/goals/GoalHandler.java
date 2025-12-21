package frc.robot.autonomous.goals;

public interface GoalHandler {
    void onStart();
    void onUpdate();
    void onStop();

    default boolean areRequirementsMet() {
        return true;
    }
}