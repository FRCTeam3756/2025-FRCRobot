// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.swerve;

import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIOInputs implements LoggableInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    @Override
    public void toLog(LogTable table) {
        table.put("DriveConnected", driveConnected);
        table.put("DrivePositionRad", drivePositionRad);
        table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
        table.put("DriveAppliedVolts", driveAppliedVolts);
        table.put("DriveCurrentAmps", driveCurrentAmps);
        table.put("TurnConnected", turnConnected);
        table.put("TurnEncoderConnected", turnEncoderConnected);
        table.put("TurnAbsolutePosition", turnAbsolutePosition);
        table.put("TurnPosition", turnPosition);
        table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec);
        table.put("TurnAppliedVolts", turnAppliedVolts);
        table.put("TurnCurrentAmps", turnCurrentAmps);
        table.put("OdometryTimestamps", odometryTimestamps);
        table.put("OdometryDrivePositionsRad", odometryDrivePositionsRad);
        table.put("OdometryTurnPositions", odometryTurnPositions);
    }

    @Override
    public void fromLog(LogTable table) {
        driveConnected = table.get("DriveConnected", driveConnected);
        drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
        driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec);
        driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
        driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
        turnConnected = table.get("TurnConnected", turnConnected);
        turnEncoderConnected = table.get("TurnEncoderConnected", turnEncoderConnected);
        turnAbsolutePosition = table.get("TurnAbsolutePosition", turnAbsolutePosition);
        turnPosition = table.get("TurnPosition", turnPosition);
        turnVelocityRadPerSec = table.get("TurnVelocityRadPerSec", turnVelocityRadPerSec);
        turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
        turnCurrentAmps = table.get("TurnCurrentAmps", turnCurrentAmps);
        odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
        odometryDrivePositionsRad = table.get("OdometryDrivePositionsRad", odometryDrivePositionsRad);
        odometryTurnPositions = table.get("OdometryTurnPositions", odometryTurnPositions);
    }
}
