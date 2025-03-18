// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.JONConstants;

public class JONSubsystem extends SubsystemBase {
    ClawSubsystem clawSubsystem = new ClawSubsystem();
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();
    private final NetworkTable networkTable;

    public JONSubsystem() {
        this.networkTable = NetworkTableInstance.getDefault().getTable(JONConstants.NETWORK_TABLE_NAME);
    }

    @Override
    public void periodic() {
        sendMatchPhaseToJON();
    }

    public void sendMatchPhaseToJON() {
        String matchPhase = DriverStation.isAutonomousEnabled() ? "Auto"
                : DriverStation.isTeleopEnabled() ? "Tele-Op" : "Pre-Match";

        sendJONData(JONConstants.NETWORK_TABLE_NAMING.MATCH_PHASE.getValue(), matchPhase);
    }

    public Object getJONData(String name, Object defaultValue) {
        NetworkTableEntry entry = this.networkTable.getEntry(name);

        if (!entry.exists()) {
            return defaultValue;
        }

        NetworkTableType type = entry.getType();

        if (type == NetworkTableType.kDouble && defaultValue instanceof Double) {
            return entry.getDouble((Double) defaultValue);
        } else if (type == NetworkTableType.kString && defaultValue instanceof String) {
            return entry.getString((String) defaultValue);
        } else if (type == NetworkTableType.kBoolean && defaultValue instanceof Boolean) {
            return entry.getBoolean((Boolean) defaultValue);
        } else if (type == NetworkTableType.kDoubleArray && defaultValue instanceof double[]) {
            return entry.getDoubleArray((double[]) defaultValue);
        } else if (type == NetworkTableType.kStringArray && defaultValue instanceof String[]) {
            return entry.getStringArray((String[]) defaultValue);
        } else if (type == NetworkTableType.kBooleanArray && defaultValue instanceof boolean[]) {
            return entry.getBooleanArray((boolean[]) defaultValue);
        } else {
            return defaultValue;
        }
    }

    public void sendJONData(String name, Object data) {
        this.networkTable.getEntry(name).setValue(data);
    }

    public void autoPickupAlgae() {
        if (clawSubsystem.isAlgaeInClaw()) {
            sendJONData(JONConstants.NETWORK_TABLE_NAMING.GOAL.getValue(),
                    JONConstants.NETWORK_TABLE_NAMING.GOAL_SCORE_ALGAE);
        } else {
            sendJONData(JONConstants.NETWORK_TABLE_NAMING.GOAL.getValue(),
                    JONConstants.NETWORK_TABLE_NAMING.GOAL_PICKUP_ALGAE);
        }
        getJONData(JONConstants.NETWORK_TABLE_NAMING.X.getValue(), 0.0);
        getJONData(JONConstants.NETWORK_TABLE_NAMING.Y.getValue(), 0.0);
        getJONData(JONConstants.NETWORK_TABLE_NAMING.ROTATION.getValue(), 0.0);
    }

    public void stopRollers() {
        clawSubsystem.stopRollers();
    }
}
