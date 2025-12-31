package frc.robot.logic.goals;

import frc.robot.logic.ControlManager;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ScoreAlgaeGoal implements GoalHandler {

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ControlManager controlManager;

    public ScoreAlgaeGoal(DrivetrainSubsystem drivetrainSubsystem, ControlManager controlManager) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.controlManager = controlManager;
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onUpdate() {
        drivetrainSubsystem.applyRequest(()
                -> drivetrainSubsystem.driveRequest
                        .withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withRotationalRate(0.0));
    }

    @Override
    public void onStop() {

    }

    @Override
    public boolean areRequirementsMet() {
        return !controlManager.isHumanDriverControl() && !controlManager.isHumanOperatorControl();
    }
}
