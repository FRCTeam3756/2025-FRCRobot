// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.hardware.OdometryConstants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.generated.LimelightHelpers.PoseEstimate;

public class OdometrySubsystem extends SubsystemBase {

    private final DrivetrainSubsystem drivetrain;

    public OdometrySubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        LimelightHelpers.setCameraPose_RobotSpace(OdometryConstants.LIMELIGHT_3G_NAME, OdometryConstants.LIMELIGHT_3G_FORWARD, OdometryConstants.LIMELIGHT_3G_RIGHT, OdometryConstants.LIMELIGHT_3G_UP, OdometryConstants.LIMELIGHT_3G_ROLL, OdometryConstants.LIMELIGHT_3G_PITCH, OdometryConstants.LIMELIGHT_3G_YAW);
        LimelightHelpers.setCameraPose_RobotSpace(OdometryConstants.LIMELIGHT_3_NAME, OdometryConstants.LIMELIGHT_3_FORWARD, OdometryConstants.LIMELIGHT_3_RIGHT, OdometryConstants.LIMELIGHT_3_UP, OdometryConstants.LIMELIGHT_3_ROLL, OdometryConstants.LIMELIGHT_3_PITCH, OdometryConstants.LIMELIGHT_3_YAW);
    }

    @Override
    public void periodic() {
        updateOdometry();

        Logger.recordOutput("Swerve/Pose", getPose());
        Logger.recordOutput("Swerve/ChassisSpeeds", getChassisSpeeds());

        Logger.recordOutput("Swerve/OdometryX", getX());
        Logger.recordOutput("Swerve/OdometryY", getY());
        Logger.recordOutput("Swerve/HeadingDeg", getRotation().getDegrees());

        Logger.recordOutput("Swerve/ModuleStates", getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        return drivetrain.getState().ModuleStates;
    }

    public double getX() {
        return drivetrain.getState().Pose.getX();
    }

    public double getY() {
        return drivetrain.getState().Pose.getY();
    }

    public Translation2d getTranslation() {
        return drivetrain.getState().Pose.getTranslation();
    }

    public Rotation2d getRotation() {
        return drivetrain.getState().Pose.getRotation();
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
                OdometryConstants.LIMELIGHT_3G_NAME,
                headingDeg,
                0, 0, 0, 0, 0
        );

        LimelightHelpers.SetRobotOrientation(
                OdometryConstants.LIMELIGHT_3_NAME,
                headingDeg, 0, 0, 0, 0, 0
        );

        PoseEstimate pose3G
                = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                        OdometryConstants.LIMELIGHT_3G_NAME
                );

        PoseEstimate pose3
                = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                        OdometryConstants.LIMELIGHT_3_NAME
                );

        PoseEstimate bestEstimate
                = selectMoreConfidentEstimate(pose3G, pose3);

        if (bestEstimate == null) {
            return;
        }

        Matrix<N3, N1> visionStdDevs = calculateDistanceBasedStdDevs(bestEstimate);

        drivetrain.addVisionMeasurement(
                bestEstimate.pose,
                bestEstimate.timestampSeconds,
                visionStdDevs
        );
    }

    private PoseEstimate selectMoreConfidentEstimate(PoseEstimate pose3, PoseEstimate pose3g) {

        if (pose3 == null && pose3g == null) {
            return null;
        }
        if (pose3 == null) {
            return (pose3g.tagCount > 0) ? pose3g : null;
        }
        if (pose3g == null) {
            return (pose3.tagCount > 0) ? pose3 : null;
        }

        boolean pose3Valid = pose3.tagCount > 0;
        boolean pose3gValid = pose3g.tagCount > 0;

        if (!pose3Valid && !pose3gValid) {
            return null;
        }
        if (pose3Valid && !pose3gValid) {
            return pose3;
        }
        if (!pose3Valid && pose3gValid) {
            return pose3g;
        }

        if (pose3.tagCount != pose3g.tagCount) {
            return (pose3.tagCount > pose3g.tagCount) ? pose3 : pose3g;
        } else {
            return pose3g;
        }
    }

    private static Matrix<N3, N1> calculateDistanceBasedStdDevs(
            PoseEstimate estimate
    ) {
        double avgDistanceMeters = 0.0;
        double avgAmbiguity = 0.0;

        for (var tag : estimate.rawFiducials) {
            avgDistanceMeters += tag.distToCamera;
            avgAmbiguity += tag.ambiguity;
        }

        int tagCount = estimate.rawFiducials.length;

        if (tagCount == 0) {
            return VecBuilder.fill(
                    OdometryConstants.MAX_XY_STD_DEV,
                    OdometryConstants.MAX_XY_STD_DEV,
                    OdometryConstants.MAX_THETA_STD_DEV
            );
        }

        avgDistanceMeters /= tagCount;
        avgAmbiguity /= tagCount;

        double distanceFactor
                = 1.0 + (avgDistanceMeters * OdometryConstants.DISTANCE_SCALING_FACTOR);

        double tagCountFactor
                = 1.0 / Math.sqrt(tagCount);

        double ambiguityFactor
                = 1.0 + avgAmbiguity;

        double xyStdDev = Math.max(
                OdometryConstants.MIN_XY_STD_DEV,
                Math.min((distanceFactor * tagCountFactor * ambiguityFactor), OdometryConstants.MAX_XY_STD_DEV)
        );

        double thetaStdDev = Math.toRadians(
                Math.min(
                        OdometryConstants.MAX_THETA_STD_DEV,
                        OdometryConstants.ANGULAR_TRUST_FACTOR * ambiguityFactor * distanceFactor
                )
        );

        return VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                thetaStdDev
        );
    }
}
