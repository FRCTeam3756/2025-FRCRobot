package frc.robot.logic.goals;

import frc.robot.logic.ControlManager;

public class IdleGoal implements GoalHandler {

    private final ControlManager controlManager;

    public IdleGoal(ControlManager controlManager) {
        this.controlManager = controlManager;
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onUpdate() {

    }

    @Override
    public void onStop() {

    }

    @Override
    public boolean areRequirementsMet() {
        return !controlManager.isHumanDriverControl() && !controlManager.isHumanOperatorControl();
    }
}
