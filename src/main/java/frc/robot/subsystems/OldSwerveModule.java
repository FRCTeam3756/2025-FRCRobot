// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.GyroConstants;
import frc.robot.constants.SwerveConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class OldSwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder absoluteEncoder;

    private final double motorOffsetRadians;
    private final boolean cancoderReversed;
    private final boolean motorReversed;

    private PIDController pidController;

    public OldSwerveModule(
        int steerCanID, int driveCanID, int cancoderID, double motorOffsetRadians,
        boolean cancoderReversed, boolean motorReversed) {
        this.driveMotor = new TalonFX(driveCanID);
        this.steerMotor = new TalonFX(steerCanID);
        this.absoluteEncoder = new CANcoder(cancoderID);

        this.motorOffsetRadians = motorOffsetRadians;
        this.cancoderReversed = cancoderReversed;
        this.motorReversed = motorReversed;

        configureMotors();
        configureEncoder();
        pidController = new PIDController(SwerveConstants.STEER_MODULE_KP, SwerveConstants.STEER_MODULE_KI, SwerveConstants.STEER_MODULE_KD);
        pidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    private void configureMotors() {
        driveMotor.setNeutralMode(DriveConstants.NEUTRAL_DRIVE_MODE);
        steerMotor.setNeutralMode(DriveConstants.NEUTRAL_DRIVE_MODE);
    }

    private void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor = new MagnetSensorConfigs();
        config.MagnetSensor.MagnetOffset = motorOffsetRadians;
        absoluteEncoder.getConfigurator().apply(config);
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getSteerPosition() {
        return steerMotor.getPosition().getValueAsDouble();
    }

    public double getAbsoluteEncoderPosition() {
        double angle = rotationsToRadians(absoluteEncoder.getPosition().getValueAsDouble()) - motorOffsetRadians;
        return angle * (cancoderReversed ? -1.0 : 1.0);
    }

    public double rotationsToRadians(double rotations) {
        return rotations * 2 * Math.PI;
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        steerMotor.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(-getSteerPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d(-getSteerPosition()).rotateBy(GyroConstants.GYRO_ANGLE_OFFSET.times(-1))
        );
    }

    public void setModuleState(SwerveModuleState desiredState) {
        double speed = Math.min(Math.abs(desiredState.speedMetersPerSecond), DriveConstants.MAX_MODULE_VELOCITY);

        double driveCommand = speed / DriveConstants.MAX_MODULE_VELOCITY;
        driveMotor.set(driveCommand * (motorReversed ? -1.0 : 1.0));

        double steerCmd = pidController.calculate(getSteerPosition(), desiredState.angle.getRadians());
        steerMotor.setVoltage(steerCmd * 12);
    }

    public void brakeMode() {
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        steerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void coastMode() {
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        steerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void stop() {
        driveMotor.set(0);
    }
}
