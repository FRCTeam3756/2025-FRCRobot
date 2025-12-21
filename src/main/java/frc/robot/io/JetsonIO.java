// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.io;

import java.util.EnumMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.connection.NetworkConstants;
import frc.robot.constants.connection.NetworkConstants.JetsonToRio;
import frc.robot.constants.connection.NetworkConstants.RioToJetson;
import frc.robot.subsystems.software.OdometrySubsystem;

public class JetsonIO {

    private String selectedAuto = "";

    private final OdometrySubsystem odometry;
    private final NetworkTable rioToJetsonTable;
    private final NetworkTable jetsonToRioTable;

    private final EnumMap<JetsonToRio, Object> commands =
            new EnumMap<>(JetsonToRio.class);

    public JetsonIO(OdometrySubsystem odometry) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        rioToJetsonTable = inst.getTable(NetworkConstants.NETWORK_TABLE_NAME);
        jetsonToRioTable = inst.getTable(NetworkConstants.NETWORK_TABLE_NAME);
        this.odometry = odometry;
    }

    public void publishRobotState() {
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
                .setDouble(odometry.getPose().getX());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_POSE_Y.key())
                .setDouble(odometry.getPose().getY());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_POSE_ROT.key())
                .setDouble(odometry.getPose().getRotation().getRadians());

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_VELOCITY_X.key())
                .setDouble(odometry.getChassisSpeeds().vxMetersPerSecond);

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_VELOCITY_Y.key())
                .setDouble(odometry.getChassisSpeeds().vyMetersPerSecond);

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_VELOCITY_ROTATIONS.key())
                .setDouble(odometry.getChassisSpeeds().omegaRadiansPerSecond);

        rioToJetsonTable
                .getEntry(RioToJetson.USER_SELECTED_AUTO.key())
                .setString(selectedAuto);
    }

    public void readJetsonCommands() {
        for (JetsonToRio command : JetsonToRio.values()) {
            var entry = jetsonToRioTable.getEntry(command.key());

            switch (command.type()) {
                case BOOLEAN ->
                    commands.put(command, entry.getBoolean(false));

                case DOUBLE ->
                    commands.put(command, entry.getDouble(0.0));

                case INTEGER ->
                    commands.put(command, entry.getInteger(0));

                case STRING ->
                    commands.put(command, entry.getString(""));

                case DOUBLE_ARRAY -> 
                        commands.put(command, entry.getDoubleArray(new double[0]));
            }
        }
    }

    public int getInteger(JetsonToRio key) {
        return (int) commands.get(key);
    }

    public double getDouble(JetsonToRio key) {
        return (double) commands.get(key);
    }

    public boolean getBoolean(JetsonToRio key) {
        return (boolean) commands.get(key);
    }

    public String getString(JetsonToRio key) {
        return (String) commands.get(key);
    }

    public double[] getDoubleArray(JetsonToRio key) {
        return (double[]) commands.get(key);
    }

    public void setSelectedAuto(String selectedAuto) {
        this.selectedAuto = selectedAuto;
    }
}
