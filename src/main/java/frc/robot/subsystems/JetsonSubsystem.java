// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.JetsonConstants;
import frc.robot.constants.JetsonConstants.JetsonToRio;
import frc.robot.constants.JetsonConstants.RioToJetson;

public class JetsonSubsystem extends SubsystemBase {
    private final NetworkTable rioToJetsonTable;
    private final NetworkTable jetsonToRioTable;

    public JetsonSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        rioToJetsonTable = inst.getTable(JetsonConstants.NETWORK_TABLE_NAME);
        jetsonToRioTable = inst.getTable(JetsonConstants.NETWORK_TABLE_NAME);
    }

    @Override
    public void periodic() {
        publishRobotState();
        readJetsonCommands();
    }

    private void publishRobotState() {
        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_ENABLED.key())
                .setBoolean(DriverStation.isEnabled());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_DISABLED.key())
                .setBoolean(DriverStation.isDisabled());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_ESTOP.key())
                .setBoolean(DriverStation.isEStopped());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_AUTO.key())
                .setBoolean(DriverStation.isAutonomousEnabled());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_TELEOP.key())
                .setBoolean(DriverStation.isTeleopEnabled());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_TEST.key())
                .setBoolean(DriverStation.isTestEnabled());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_ALLIANCE.key())
                .setString(
                        DriverStation.getAlliance().isPresent()
                                ? DriverStation.getAlliance().get().name().toLowerCase()
                                : "unknown");

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_MATCH_TIME.key())
                .setDouble(DriverStation.getMatchTime());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_START_LOCATION.key())
                .setInteger(DriverStation.getLocation().getAsInt());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_POSE_X.key())
                .setDouble(DriverStation.getMatchTime()); //TODO: replace with actual value

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_POSE_Y.key())
                .setDouble(DriverStation.getMatchTime()); //TODO: replace with actual value

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_POSE_ROT.key())
                .setDouble(DriverStation.getMatchTime()); //TODO: replace with actual value

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_VELOCITY_X.key())
                .setDouble(DriverStation.getMatchTime()); //TODO: replace with actual value

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_VELOCITY_Y.key())
                .setDouble(DriverStation.getMatchTime()); //TODO: replace with actual value

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_VELOCITY_ANGLE.key())
                .setDouble(DriverStation.getMatchTime()); //TODO: replace with actual value
    }

    public Map<JetsonToRio, Object> readJetsonCommands() {
        EnumMap<JetsonToRio, Object> data = new EnumMap<>(JetsonToRio.class);

        data.put(JetsonToRio.CMD_VX,
                jetsonToRioTable.getEntry(JetsonToRio.CMD_VX.key()).getDouble(0.0));

        data.put(JetsonToRio.CMD_VY,
                jetsonToRioTable.getEntry(JetsonToRio.CMD_VY.key()).getDouble(0.0));

        data.put(JetsonToRio.CMD_OMEGA,
                jetsonToRioTable.getEntry(JetsonToRio.CMD_OMEGA.key()).getDouble(0.0));

        data.put(JetsonToRio.CMD_TURBO,
                jetsonToRioTable.getEntry(JetsonToRio.CMD_TURBO.key()).getBoolean(false));

        data.put(JetsonToRio.CMD_INTAKE,
                jetsonToRioTable.getEntry(JetsonToRio.CMD_INTAKE.key()).getBoolean(false));

        data.put(JetsonToRio.CMD_OUTTAKE,
                jetsonToRioTable.getEntry(JetsonToRio.CMD_OUTTAKE.key()).getBoolean(false));

        data.put(JetsonToRio.CMD_CLIMB,
                jetsonToRioTable.getEntry(JetsonToRio.CMD_CLIMB.key()).getBoolean(false));

        data.put(JetsonToRio.CMD_ELEVATOR_SETPOINT,
                jetsonToRioTable.getEntry(JetsonToRio.CMD_ELEVATOR_SETPOINT.key()).getDouble(0.0));

        return data;
    }
}
