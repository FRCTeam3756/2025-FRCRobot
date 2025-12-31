// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.connection.FieldObjectConstants.Algae;
import frc.robot.constants.connection.FieldObjectConstants.Coral;
import frc.robot.constants.connection.FieldObjectConstants.FieldObject;
import frc.robot.constants.connection.FieldObjectConstants.OpponentRobot;
import frc.robot.constants.connection.FieldObjectConstants.TeammateRobot;
import frc.robot.constants.connection.NetworkConstants;
import frc.robot.constants.connection.NetworkConstants.JetsonToRio;
import frc.robot.constants.hardware.ChassisConstants;
import frc.robot.constants.logic.OtherRobotConstants;

public class JetsonSubsystem extends SubsystemBase {
    
    private final OdometrySubsystem odometrySubsystem;
    private final NetworkTable jetsonToRioTable;

    private final EnumMap<JetsonToRio, Object> commands
            = new EnumMap<>(JetsonToRio.class);

    public JetsonSubsystem(OdometrySubsystem odometrySubsystem) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        jetsonToRioTable = inst.getTable(NetworkConstants.NETWORK_TABLE_NAME);
        this.odometrySubsystem = odometrySubsystem;
    }

    @Override
    public void periodic() {
        readJetsonCommands();
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

    public List<OpponentRobot> getOpponentRobots() {
        String[] cameras = getStringArray(JetsonToRio.OPPONENT_CAMERA);
        double[] distances = getDoubleArray(JetsonToRio.OPPONENT_DISTANCE);
        double[] angles = getDoubleArray(JetsonToRio.OPPONENT_ANGLE);
        double[] timestamps = getDoubleArray(JetsonToRio.OPPONENT_TIMESTAMP);

        List<OpponentRobot> list = new ArrayList<>();

        for (int i = 0; i < distances.length; i++) {
            Transform2d camTransform = ChassisConstants.CAMERA_TRANSFORMS.get(cameras[i]);
            if (camTransform == null) {
                continue;
            }

            Translation2d fieldPos = cameraDetectionToField(
                    camTransform,
                    distances[i],
                    angles[i]
            );

            list.add(new OpponentRobot(cameras[i], fieldPos.getX(), fieldPos.getY(), distances[i], angles[i], timestamps[i]));
        }
        return list;
    }

    public List<TeammateRobot> getTeammateRobots() {
        String[] cameras = getStringArray(JetsonToRio.TEAMMATE_CAMERA);
        double[] distances = getDoubleArray(JetsonToRio.TEAMMATE_DISTANCE);
        double[] angles = getDoubleArray(JetsonToRio.TEAMMATE_ANGLE);
        double[] timestamps = getDoubleArray(JetsonToRio.TEAMMATE_TIMESTAMP);

        List<TeammateRobot> list = new ArrayList<>();

        for (int i = 0; i < distances.length; i++) {
            Transform2d camTransform = ChassisConstants.CAMERA_TRANSFORMS.get(cameras[i]);
            if (camTransform == null) {
                continue;
            }

            Translation2d fieldPos = cameraDetectionToField(
                    camTransform,
                    distances[i],
                    angles[i]
            );

            list.add(new TeammateRobot(cameras[i], fieldPos.getX(), fieldPos.getY(), distances[i], angles[i], timestamps[i]));
        }
        return list;
    }

    public List<Algae> getAlgae() {
        String[] cameras = getStringArray(JetsonToRio.ALGAE_CAMERA);
        double[] distances = getDoubleArray(JetsonToRio.ALGAE_DISTANCE);
        double[] angles = getDoubleArray(JetsonToRio.ALGAE_ANGLE);
        double[] timestamps = getDoubleArray(JetsonToRio.ALGAE_TIMESTAMP);

        List<Algae> list = new ArrayList<>();

        for (int i = 0; i < distances.length; i++) {
            Transform2d camTransform = ChassisConstants.CAMERA_TRANSFORMS.get(cameras[i]);
            if (camTransform == null) {
                continue;
            }

            Translation2d fieldPos = cameraDetectionToField(
                    camTransform,
                    distances[i],
                    angles[i]
            );

            list.add(new Algae(cameras[i], fieldPos.getX(), fieldPos.getY(), distances[i], angles[i], timestamps[i]));
        }
        return list;
    }

    public List<Coral> getCorals() {
        String[] cameras = getStringArray(JetsonToRio.CORAL_CAMERA);
        double[] distances = getDoubleArray(JetsonToRio.CORAL_DISTANCE);
        double[] angles = getDoubleArray(JetsonToRio.CORAL_ANGLE);
        double[] timestamps = getDoubleArray(JetsonToRio.CORAL_TIMESTAMP);

        List<Coral> list = new ArrayList<>();

        for (int i = 0; i < distances.length; i++) {
            Transform2d camTransform = ChassisConstants.CAMERA_TRANSFORMS.get(cameras[i]);
            if (camTransform == null) {
                continue;
            }

            Translation2d fieldPos = cameraDetectionToField(
                    camTransform,
                    distances[i],
                    angles[i]
            );
            
            list.add(new Coral(cameras[i], fieldPos.getX(), fieldPos.getY(), distances[i], angles[i], timestamps[i]));
        }
        return list;
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

    private String[] getStringArray(JetsonToRio key) {
        return (String[]) commands.get(key);
    }

    public double[] getDoubleArray(JetsonToRio key) {
        return (double[]) commands.get(key);
    }

    private Translation2d cameraDetectionToField(
            Transform2d cameraToRobot,
            double distanceMeters,
            double angleRadians
    ) {
        Translation2d cameraRelative = new Translation2d(
                distanceMeters * Math.cos(angleRadians),
                distanceMeters * Math.sin(angleRadians)
        );

        Translation2d robotRelative
                = cameraRelative.rotateBy(cameraToRobot.getRotation())
                        .plus(cameraToRobot.getTranslation());

        return robotRelative.rotateBy(odometrySubsystem.getRotation())
                .plus(odometrySubsystem.getTranslation());
    }


    public List<Pair<Translation2d, Translation2d>> getDynamicObstacles() {
        List<Pair<Translation2d, Translation2d>> obstacleLocations = new ArrayList<>();

        List<FieldObject> obstacles = Stream.concat(
                getOpponentRobots().stream(),
                getTeammateRobots().stream()
        ).toList();

        for (int i = 0; i < obstacles.size(); i++) {
            double centerX = obstacles.get(i).x;
            double centerY = obstacles.get(i).y;

            Translation2d minCorner = new Translation2d(
                    centerX - OtherRobotConstants.MAX_ROBOT_WIDTH / 2.0,
                    centerY - OtherRobotConstants.MAX_ROBOT_LENGTH / 2.0
            );

            Translation2d maxCorner = new Translation2d(
                    centerX + OtherRobotConstants.MAX_ROBOT_WIDTH / 2.0,
                    centerY + OtherRobotConstants.MAX_ROBOT_LENGTH / 2.0
            );

            obstacleLocations.add(new Pair<>(minCorner, maxCorner));
        }

        return obstacleLocations;
    }

    public List<Translation2d> getOpponentRobotLocations() {
        List<Translation2d> opponentRobotLocations = new ArrayList<>();
        List<OpponentRobot> opponentRobots = getOpponentRobots();

        for (int i = 0; i < opponentRobots.size(); i++) {
            double centerX = opponentRobots.get(i).x;
            double centerY = opponentRobots.get(i).y;

            opponentRobotLocations.add(new Translation2d(centerX, centerY));
        }

        return opponentRobotLocations;
    }
}
