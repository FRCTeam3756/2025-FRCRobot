package frc.robot.autonomous.goals;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.connection.FieldObjectConstants.Algae;
import frc.robot.constants.subsystems.VisionConstants;
import frc.robot.subsystems.mechanical.DrivetrainSubsystem;
import frc.robot.subsystems.software.VisionSubsystem;

public class FindAlgaeGoal implements GoalHandler {

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final VisionSubsystem visionSubsystem;

    public FindAlgaeGoal(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onUpdate() {
        List<Algae> algaeList = visionSubsystem.getAlgae(drivetrainSubsystem.getState().Pose);

        if (algaeList.isEmpty()) {
            return;
        }

        Pose2d robotPose = drivetrainSubsystem.getState().Pose;
        ChassisSpeeds chassisSpeeds = drivetrainSubsystem.getState().Speeds;

        int selectedAlgaeIndex = selectAlgaeIndex(robotPose, chassisSpeeds, algaeList);
        if (selectedAlgaeIndex == -1) {
            return;
        }

        Algae goalAlgae = algaeList.get(selectedAlgaeIndex);
        Translation2d algaeLocation = new Translation2d(goalAlgae.x, goalAlgae.y);

        double distanceToAlgae = algaeLocation.getDistance(robotPose.getTranslation());

        if (distanceToAlgae <= VisionConstants.ALGAE_INTAKE_DISTANCE) {
            visionSubsystem.applyClawCommands(true, false);
        }

        if (distanceToAlgae > VisionConstants.ALGAE_FINE_TUNE_DISTANCE) {
            double deltaX = algaeLocation.getX() - robotPose.getX();
            double deltaY = algaeLocation.getY() - robotPose.getY();
            double goalTheta = new Rotation2d(deltaX, deltaY).getRadians();
            visionSubsystem.applyDriveCommands(algaeLocation.getX(), algaeLocation.getY(), goalTheta, VisionConstants.ALGAE_FINE_TUNE_FORWARD_GAIN);
        } else {
            double vx = clamp(goalAlgae.distance * VisionConstants.ALGAE_FINE_TUNE_FORWARD_GAIN,
                              VisionConstants.MINIMUM_DRIVE_SPEED,
                              VisionConstants.MAX_DRIVE_SPEED);
            double omega = clamp(goalAlgae.angle * VisionConstants.ALGAE_FINE_TUNE_ROTATIONAL_GAIN,
                                 -VisionConstants.MAX_ROTATIONAL_RATE,
                                 VisionConstants.MAX_ROTATIONAL_RATE);

            drivetrainSubsystem.applyRequest(() -> drivetrainSubsystem.driveRequest
                    .withVelocityX(vx)
                    .withVelocityY(0.0)
                    .withRotationalRate(omega));
        }
    }

    @Override
    public void onStop() {

    }

    @Override
    public boolean areRequirementsMet() {
        return !visionSubsystem.isHumanDriverControl() && !visionSubsystem.isHumanOperatorControl();
    }

    private int selectAlgaeIndex(Pose2d robotPose, ChassisSpeeds chassisSpeeds, List<Algae> algaeList) {
        if (algaeList.isEmpty()) return -1;

        double vx = chassisSpeeds.vxMetersPerSecond;
        double vy = chassisSpeeds.vyMetersPerSecond;

        int bestIndex = 0;
        double bestScore = Double.MAX_VALUE;

        for (int i = 0; i < algaeList.size(); i++) {
            Algae algae = algaeList.get(i);
            Translation2d algaeLocation = new Translation2d(algae.x, algae.y);

            double distance = algaeLocation.getDistance(robotPose.getTranslation());
            double toAlgaeX = algaeLocation.getX() - robotPose.getX();
            double toAlgaeY = algaeLocation.getY() - robotPose.getY();
            double dot = vx * toAlgaeX + vy * toAlgaeY;

            double weightedValue = (distance * VisionConstants.ALGAE_SELECTION_PROXIMITY_WEIGHT)
                                - (dot * VisionConstants.ALGAE_SELECTION_VELOCITY_WEIGHT);

            if (weightedValue < bestScore) {
                bestScore = weightedValue;
                bestIndex = i;
            }
        }

        return bestIndex;
}

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}