// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputs inputs = new ModuleIOInputs();
  private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    this.io = io;
    this.constants = constants;
    driveDisconnectedAlert = new Alert(
        "Disconnected drive motor on module " + Integer.toString(index) + ".",
        AlertType.kError);
    turnDisconnectedAlert = new Alert(
        "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert = new Alert(
        "Disconnected turn encoder on module " + Integer.toString(index) + ".",
        AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);

    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  public void runSetpoint(SwerveModuleState state) {
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
