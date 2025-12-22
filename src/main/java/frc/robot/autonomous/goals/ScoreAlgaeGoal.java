package frc.robot.autonomous.goals;

import frc.robot.subsystems.mechanical.DrivetrainSubsystem;
import frc.robot.subsystems.software.VisionSubsystem;

public class ScoreAlgaeGoal implements GoalHandler {

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final VisionSubsystem visionSubsystem;

    public ScoreAlgaeGoal(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.visionSubsystem = visionSubsystem;
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
        return !visionSubsystem.isHumanDriverControl() && !visionSubsystem.isHumanOperatorControl();
    }
}
