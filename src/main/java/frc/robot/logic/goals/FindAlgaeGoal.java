package frc.robot.logic.goals;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.claw.RunRollersCommand;
import frc.robot.commands.drivetrain.DriveToPoseCommand;
import frc.robot.constants.connection.FieldObjectConstants.Algae;
import frc.robot.constants.logic.FineTuningConstants;
import frc.robot.logic.ControlManager;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamepieceSensorSubsystem;
import frc.robot.subsystems.JetsonSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

public class FindAlgaeGoal implements GoalHandler {

    private final ControlManager controlManager;

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final OdometrySubsystem odometrySubsystem;
    private final JetsonSubsystem jetsonSubsystem;
    private final GamepieceSensorSubsystem gamepieceSensorSubsystem;

    public FindAlgaeGoal(ControlManager controlManager, ClawSubsystem clawSubsystem, DrivetrainSubsystem drivetrainSubsystem, OdometrySubsystem odometrySubsystem, JetsonSubsystem jetsonSubsystem, GamepieceSensorSubsystem gamepieceSensorSubsystem) {
        this.controlManager = controlManager;
        this.clawSubsystem = clawSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.jetsonSubsystem = jetsonSubsystem;
        this.gamepieceSensorSubsystem = gamepieceSensorSubsystem;
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onUpdate() {
        List<Algae> algaeList = jetsonSubsystem.getAlgae();

        if (algaeList.isEmpty()) {
            return;
        }

        int selectedAlgaeIndex = selectAlgaeIndex(odometrySubsystem.getPose(), odometrySubsystem.getChassisSpeeds(), algaeList);
        if (selectedAlgaeIndex == -1) {
            return;
        }

        Algae goalAlgae = algaeList.get(selectedAlgaeIndex);
        Translation2d algaeLocation = new Translation2d(goalAlgae.x, goalAlgae.y);

        double distanceToAlgae = algaeLocation.getDistance(odometrySubsystem.getTranslation());

        if (distanceToAlgae <= FineTuningConstants.ALGAE_INTAKE_DISTANCE) {
            new RunRollersCommand(clawSubsystem, true, false).schedule();
        }

        if (distanceToAlgae > FineTuningConstants.ALGAE_FINE_TUNE_DISTANCE) {
            double deltaX = algaeLocation.getX() - odometrySubsystem.getX();
            double deltaY = algaeLocation.getY() - odometrySubsystem.getY();
            Pose2d targetPose = new Pose2d(new Translation2d(deltaX, deltaY), new Rotation2d(deltaX, deltaY));
            new InstantCommand(() -> new DriveToPoseCommand(
                odometrySubsystem, drivetrainSubsystem, jetsonSubsystem, 
                targetPose, FineTuningConstants.ALGAE_FINE_TUNE_FORWARD_GAIN)
            ).schedule();
        } else {
            double vx = clamp(goalAlgae.distance * FineTuningConstants.ALGAE_FINE_TUNE_FORWARD_GAIN,
                    FineTuningConstants.MINIMUM_DRIVE_SPEED,
                    FineTuningConstants.MAX_DRIVE_SPEED);
            double omega = clamp(goalAlgae.angle * FineTuningConstants.ALGAE_FINE_TUNE_ROTATIONAL_GAIN,
                    -FineTuningConstants.MAX_ROTATIONAL_RATE,
                    FineTuningConstants.MAX_ROTATIONAL_RATE);

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
        return !controlManager.isHumanDriverControl() && !controlManager.isHumanOperatorControl();
    }

    @Override
    public boolean isFinished() {
        return gamepieceSensorSubsystem.hasAlgae();
    }

    private int selectAlgaeIndex(Pose2d robotPose, ChassisSpeeds chassisSpeeds, List<Algae> algaeList) {
        if (algaeList.isEmpty()) {
            return -1;
        }

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

            double weightedValue = (distance * FineTuningConstants.ALGAE_SELECTION_PROXIMITY_WEIGHT)
                    - (dot * FineTuningConstants.ALGAE_SELECTION_VELOCITY_WEIGHT);

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
