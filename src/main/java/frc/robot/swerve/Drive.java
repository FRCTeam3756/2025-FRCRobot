// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;

public class Drive extends SubsystemBase {
  static final double ODOMETRY_FREQUENCY =
      new CANBus(SwerveConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(SwerveConstants.FL_SWERVE_MODULE.LocationX, SwerveConstants.FL_SWERVE_MODULE.LocationY),
              Math.hypot(SwerveConstants.FR_SWERVE_MODULE.LocationX, SwerveConstants.FR_SWERVE_MODULE.LocationY)),
          Math.max(
              Math.hypot(SwerveConstants.BL_SWERVE_MODULE.LocationX, SwerveConstants.BL_SWERVE_MODULE.LocationY),
              Math.hypot(SwerveConstants.BR_SWERVE_MODULE.LocationX, SwerveConstants.BR_SWERVE_MODULE.LocationY)));

  static final Lock odometryLock = new ReentrantLock();
  private final Module[] modules = new Module[4];

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private final Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive() {
    modules[0] = new Module(new ModuleIOTalonFX(SwerveConstants.FL_SWERVE_MODULE), 0, SwerveConstants.FL_SWERVE_MODULE);
    modules[1] = new Module(new ModuleIOTalonFX(SwerveConstants.FR_SWERVE_MODULE), 1, SwerveConstants.FR_SWERVE_MODULE);
    modules[2] = new Module(new ModuleIOTalonFX(SwerveConstants.BL_SWERVE_MODULE), 2, SwerveConstants.BL_SWERVE_MODULE);
    modules[3] = new Module(new ModuleIOTalonFX(SwerveConstants.BR_SWERVE_MODULE), 3, SwerveConstants.BR_SWERVE_MODULE);
  }

  public void initialize() {
    PhoenixOdometryThread.getInstance().start();
  }

  @Override
  public void periodic() {
    odometryLock.lock();
      try {
          for (var module : modules) {
              module.periodic();
          } } finally {
          odometryLock.unlock();
      }

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.SPEED_AT_12_VOLTS);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond);
  }

  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(SwerveConstants.FL_X_POSITION, SwerveConstants.FL_Y_POSITION),
      new Translation2d(SwerveConstants.FR_X_POSITION, SwerveConstants.FR_Y_POSITION),
      new Translation2d(SwerveConstants.BL_X_POSITION, SwerveConstants.BL_Y_POSITION),
      new Translation2d(SwerveConstants.BR_X_POSITION, SwerveConstants.BR_Y_POSITION)
    };
  }
}
