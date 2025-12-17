// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.LimelightHelpers;

public class OdometrySubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    public OdometrySubsystem(CommandSwerveDrivetrain drivetrain) {
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
        LimelightHelpers.SetRobotOrientation("limelight", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        boolean reject = false;
        if (mt2.tagCount == 0) {
            reject = true;
        }
        if (!reject) {
            drivetrain.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds,
                    VecBuilder.fill(0.7, 0.7, 9999999)
            );
        }
    }
}
