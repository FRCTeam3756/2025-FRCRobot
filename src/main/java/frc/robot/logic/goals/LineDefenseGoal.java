package frc.robot.logic.goals;

import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.logic.FineTuningConstants;
import frc.robot.logic.ControlManager;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.JetsonSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

public class LineDefenseGoal implements GoalHandler {

    private final ControlManager controlManager;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final OdometrySubsystem odometrySubsystem;
    private final JetsonSubsystem jetsonSubsystem;

    private Translation2d lineDefenseOrigin;
    private Rotation2d lineDefenseAngle;

    public LineDefenseGoal(ControlManager controlManager, DrivetrainSubsystem drivetrainSubsystem, OdometrySubsystem odometrySubsystem, JetsonSubsystem jetsonSubsystem) {
        this.controlManager = controlManager;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.jetsonSubsystem = jetsonSubsystem;
    }

    @Override
    public void onStart() {
        lineDefenseOrigin = odometrySubsystem.getTranslation();
        lineDefenseAngle = odometrySubsystem.getRotation().plus(Rotation2d.kCW_90deg);
    }

    @Override
    public void onUpdate() {
        List<Translation2d> opponentLocations = jetsonSubsystem.getOpponentRobotLocations();
        if (opponentLocations.isEmpty()) {
            drivetrainSubsystem.applyRequest(() -> drivetrainSubsystem.driveRequest
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0));
            return;
        }

        Translation2d robotPos = odometrySubsystem.getTranslation();

        Translation2d opponentLocation = opponentLocations.stream()
                .min(Comparator.comparingDouble(opp -> lineDefenseOpponentThreatScore(opp, robotPos)))
                .get();

        Translation2d lineDir = new Translation2d(
                Math.cos(lineDefenseAngle.getRadians()),
                Math.sin(lineDefenseAngle.getRadians())
        );

        Translation2d originToOpponent = opponentLocation.minus(lineDefenseOrigin);

        double proj
                = originToOpponent.getX() * lineDir.getX()
                + originToOpponent.getY() * lineDir.getY();

        Translation2d interceptPoint
                = lineDefenseOrigin.plus(lineDir.times(proj));

        double opponentDistance = opponentLocation.getDistance(robotPos);

        Translation2d desiredPosition = interceptPoint;
        if (opponentDistance < FineTuningConstants.LINE_DEFENSE_PUSH_RESPONSE_DISTANCE) {
            Translation2d pushVector = opponentLocation.minus(robotPos).times(FineTuningConstants.LINE_DEFENSE_AGGRESSION_GAIN * Math.max(1.0, 0.6 / opponentDistance));
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
                    = desiredHeading.minus(odometrySubsystem.getRotation());

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
        return !controlManager.isHumanDriverControl() && !controlManager.isHumanOperatorControl();
    }

    private double lineDefenseOpponentThreatScore(Translation2d opponentPos, Translation2d robotPos) {
        double distToLine = distanceToLine(opponentPos);
        double distToRobot = opponentPos.getDistance(robotPos);

        double lineWeight = 2.0;
        double robotWeight = 1.0;

        return lineWeight * distToLine + robotWeight * distToRobot;
    }

    private double distanceToLine(Translation2d point) {
        Translation2d dir = new Translation2d(Math.cos(lineDefenseAngle.getRadians()),
                Math.sin(lineDefenseAngle.getRadians()));

        Translation2d diff = point.minus(lineDefenseOrigin);
        double cross = diff.getX() * dir.getY() - diff.getY() * dir.getX();
        return Math.abs(cross) / dir.getNorm();
    }

}
