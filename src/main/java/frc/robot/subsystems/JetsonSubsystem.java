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
    private String selectedAuto = "";
    private final OdometrySubsystem odometry;
    private final NetworkTable rioToJetsonTable;
    private final NetworkTable jetsonToRioTable;

    public JetsonSubsystem(OdometrySubsystem odometry) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        rioToJetsonTable = inst.getTable(JetsonConstants.NETWORK_TABLE_NAME);
        jetsonToRioTable = inst.getTable(JetsonConstants.NETWORK_TABLE_NAME);
        this.odometry = odometry;
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
                .setDouble(odometry.getChassisSpeeds().vxMetersPerSecond);

        rioToJetsonTable
                .getEntry(RioToJetson.ROBOT_VELOCITY_ROTATIONS.key())
                .setDouble(odometry.getChassisSpeeds().omegaRadiansPerSecond);

        rioToJetsonTable
                .getEntry(RioToJetson.USER_SELECTED_AUTO.key())
                .setString(selectedAuto);
    }

    public Map<JetsonToRio, Object> readJetsonCommands() {
        EnumMap<JetsonToRio, Object> data = new EnumMap<>(JetsonToRio.class);

        for (JetsonToRio command : JetsonToRio.values()) {
            var entry = jetsonToRioTable.getEntry(command.key());

            switch (command.type()) {
                case BOOLEAN ->
                    data.put(command, entry.getBoolean(false));

                case DOUBLE ->
                    data.put(command, entry.getDouble(0.0));

                case STRING ->
                    data.put(command, entry.getString(""));
            }
        }

        return data;
    }

    public void setSelectedAuto(String selectedAuto) {
        this.selectedAuto = selectedAuto;
    }
}
