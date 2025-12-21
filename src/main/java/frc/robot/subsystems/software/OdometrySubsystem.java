// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.subsystems.software;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.subsystems.VisionConstants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.mechanical.DrivetrainSubsystem;

public class OdometrySubsystem extends SubsystemBase {

    private final DrivetrainSubsystem drivetrain;
    private String activeLimelight = null;

    public OdometrySubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return drivetrain.getState().Speeds;
    }

    public void updateOdometry() {
        double headingDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();

        LimelightHelpers.SetRobotOrientation(
                VisionConstants.LIMELIGHT_3G_NAME,
                headingDeg,
                0, 0, 0, 0, 0
        );

        LimelightHelpers.SetRobotOrientation(
                VisionConstants.LIMELIGHT_3_NAME,
                headingDeg, 0, 0, 0, 0, 0
        );

        LimelightHelpers.PoseEstimate pose3G
                = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                        VisionConstants.LIMELIGHT_3G_NAME
                );

        LimelightHelpers.PoseEstimate pose3
                = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                        VisionConstants.LIMELIGHT_3_NAME
                );

        LimelightHelpers.PoseEstimate bestEstimate
                = selectMoreConfidentEstimate(pose3G, pose3);

        if (bestEstimate == null) {
            return;
        }

        var visionStdDevs = calculateDistanceBasedStdDevs(bestEstimate);

        drivetrain.addVisionMeasurement(
                bestEstimate.pose,
                bestEstimate.timestampSeconds,
                visionStdDevs
        );

        drivetrain.registerTelemetry(null);
    }

    private LimelightHelpers.PoseEstimate selectMoreConfidentEstimate(LimelightHelpers.PoseEstimate limelight3, LimelightHelpers.PoseEstimate limelight3g) {

        if (limelight3 == null && limelight3g == null) {
            return null;
        }
        if (limelight3 == null) {
            return (limelight3g.tagCount > 0) ? limelight3g : null;
        }
        if (limelight3g == null) {
            return (limelight3.tagCount > 0) ? limelight3 : null;
        }

        boolean limelight3Valid = limelight3.tagCount > 0;
        boolean limelight3gValid = limelight3g.tagCount > 0;

        if (!limelight3Valid && !limelight3gValid) {
            return null;
        }
        if (limelight3Valid && !limelight3gValid) {
            return limelight3;
        }
        if (!limelight3Valid && limelight3gValid) {
            return limelight3g;
        }

        if (limelight3.tagCount != limelight3g.tagCount) {
            return (limelight3.tagCount > limelight3g.tagCount) ? limelight3 : limelight3g;
        }

        return (limelight3.timestampSeconds > limelight3g.timestampSeconds) ? limelight3 : limelight3g;
    }

    private static Matrix<N3, N1> calculateDistanceBasedStdDevs(
            LimelightHelpers.PoseEstimate estimate
    ) {
        double avgDistanceMeters = 0.0;
        double avgAmbiguity = 0.0;

        for (var tag : estimate.rawFiducials) {
            avgDistanceMeters += tag.distToCamera;
            avgAmbiguity += tag.ambiguity;
        }

        int tagCount = estimate.rawFiducials.length;

        avgDistanceMeters /= tagCount;
        avgAmbiguity /= tagCount;

        double distanceFactor
                = 1.0 + (avgDistanceMeters * VisionConstants.VISION_DISTANCE_SCALING_FACTOR);

        double tagCountFactor
                = 1.0 / Math.sqrt(tagCount);

        double ambiguityFactor
                = 1.0 + avgAmbiguity;

        double xyStdDev
                = distanceFactor * tagCountFactor * ambiguityFactor;

        xyStdDev = Math.max(
                VisionConstants.MIN_VISION_STD_DEV,
                Math.min(xyStdDev, VisionConstants.MAX_VISION_STD_DEV)
        );

        return VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                999999.0
        );
    }

    public void setActiveLimelight(String limelight) {
        activeLimelight = limelight;
    }

    public String getActiveLimelight() {
        return activeLimelight;
    }
}
