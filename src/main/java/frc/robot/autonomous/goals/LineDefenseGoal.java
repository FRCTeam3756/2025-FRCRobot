package frc.robot.autonomous.goals;

import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.subsystems.VisionConstants;
import frc.robot.subsystems.mechanical.DrivetrainSubsystem;
import frc.robot.subsystems.software.VisionSubsystem;

public class LineDefenseGoal implements GoalHandler {

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final VisionSubsystem visionSubsystem;
    private Pose2d lineDefenseOrigin;
    private Rotation2d lineDefenseAngle;

    public LineDefenseGoal(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void onStart() {
        lineDefenseOrigin = drivetrainSubsystem.getState().Pose;
        lineDefenseAngle = lineDefenseOrigin.getRotation().plus(Rotation2d.kCW_90deg);
    }

    @Override
    public void onUpdate() {
        List<Translation2d> opponentLocations = visionSubsystem.getOpponentRobotLocations();
        if (opponentLocations.isEmpty()) {
            drivetrainSubsystem.applyRequest(() -> drivetrainSubsystem.driveRequest
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0));
            return;
        }

        Pose2d robotPose = drivetrainSubsystem.getState().Pose;
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d opponentLocation = opponentLocations.stream()
                .min(Comparator.comparingDouble(opp -> lineDefenseOpponentThreatScore(opp, robotPos)))
                .get();

        Translation2d lineDir = new Translation2d(
                Math.cos(lineDefenseAngle.getRadians()),
                Math.sin(lineDefenseAngle.getRadians())
        );

        Translation2d originToOpponent = opponentLocation.minus(lineDefenseOrigin.getTranslation());

        double proj
                = originToOpponent.getX() * lineDir.getX()
                + originToOpponent.getY() * lineDir.getY();

        Translation2d interceptPoint
                = lineDefenseOrigin.getTranslation().plus(lineDir.times(proj));

        double opponentDistance = opponentLocation.getDistance(robotPos);

        Translation2d desiredPosition = interceptPoint;
        if (opponentDistance < VisionConstants.LINE_DEFENSE_PUSH_RESPONSE_DISTANCE) {
            Translation2d pushVector = opponentLocation.minus(robotPos).times(VisionConstants.LINE_DEFENSE_AGGRESSION_GAIN * Math.max(1.0, 0.6 / opponentDistance));
            desiredPosition = interceptPoint.plus(pushVector);
        }

        Translation2d error = desiredPosition.minus(robotPos);

        Translation2d desiredVelocity
                = error.div(Math.max(error.getNorm(), 0.001))
                        .times(Math.min(error.getNorm() * drivetrainSubsystem.maxSpeed / 1.5, drivetrainSubsystem.maxSpeed));

        double omega;
        if (opponentDistance > 0.4) {
            Rotation2d desiredHeading
                    = opponentLocation.minus(robotPos).getAngle();

            Rotation2d headingError
                    = desiredHeading.minus(robotPose.getRotation());

            omega
                    = Math.max(
                            -drivetrainSubsystem.maxAngularRate,
                            Math.min(drivetrainSubsystem.maxAngularRate, headingError.getRadians() * drivetrainSubsystem.maxAngularRate / Math.PI)
                    );
        } else {
            omega = 0.0;
        }

        drivetrainSubsystem.applyRequest(() -> drivetrainSubsystem.driveRequest
                .withVelocityX(desiredVelocity.getX())
                .withVelocityY(desiredVelocity.getY())
                .withRotationalRate(omega));
    }

    @Override
    public void onStop() {
        drivetrainSubsystem.applyRequest(() -> drivetrainSubsystem.driveRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
    }

    @Override
    public boolean areRequirementsMet() {
        return !visionSubsystem.isHumanDriverControl() && !visionSubsystem.isHumanOperatorControl();
    }

    private double lineDefenseOpponentThreatScore(Translation2d opponentPos, Translation2d robotPos) {
        double distToLine = distanceToLine(opponentPos);
        double distToRobot = opponentPos.getDistance(robotPos);

        double lineWeight = 2.0;
        double robotWeight = 1.0;

        return lineWeight * distToLine + robotWeight * distToRobot;
    }

    private double distanceToLine(Translation2d point) {
        Translation2d origin = lineDefenseOrigin.getTranslation();
        Translation2d dir = new Translation2d(Math.cos(lineDefenseAngle.getRadians()),
                Math.sin(lineDefenseAngle.getRadians()));

        Translation2d diff = point.minus(origin);
        double cross = diff.getX() * dir.getY() - diff.getY() * dir.getX();
        return Math.abs(cross) / dir.getNorm();
    }

}
