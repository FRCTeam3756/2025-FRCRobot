// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.swerve;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
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
