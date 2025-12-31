package frc.robot.constants.connection;

public class NetworkConstants {
    public static final String ROBOT_IP_ADDRESS = "10.37.56.2";
    public static final String NETWORK_TABLE_NAME = "AIPipeline";
    public static final String DATA_ENTRY_NAME = "data";

    public enum JetsonToRio {
        DESIRED_X("jetson/command/x", EntryType.DOUBLE),
        DESIRED_Y("jetson/command/y", EntryType.DOUBLE),
        DESIRED_THETA("jetson/command/theta", EntryType.DOUBLE),
        DESIRED_TURBO_STATE("jetson/command/turbo", EntryType.BOOLEAN),
        DESIRED_INTAKE("jetson/command/intake", EntryType.BOOLEAN),
        DESIRED_OUTTAKE("jetson/command/outtake", EntryType.BOOLEAN),
        DESIRED_SHOOT("jetson/command/shoot", EntryType.BOOLEAN),
        DESIRED_SHOOT_SPEED("jetson/command/shoot_rpm", EntryType.DOUBLE),
        DESIRED_ELEVATOR_SETPOINT("jetson/command/elevator_setpoint", EntryType.INTEGER),
        DESIRED_WRIST_ANGLE("jetson/command/wrist_angle", EntryType.DOUBLE),
        DESIRED_CLIMB("jetson/command/climb", EntryType.BOOLEAN),
        PATH_ACTIVE("jetson/path/active", EntryType.BOOLEAN),
        PATH_NAME("jetson/path/name", EntryType.STRING),
        PATH_PROGRESS("jetson/path/progress", EntryType.DOUBLE),
        PATH_COMPLETE("jetson/path/complete", EntryType.BOOLEAN),
        AUTO_OBJECTIVE("jetson/strategy/objective", EntryType.STRING),
        AUTO_SUBTARGET("jetson/strategy/subtarget", EntryType.STRING),
        AUTO_PRIORITY("jetson/strategy/priority", EntryType.DOUBLE),

        MESSAGE("jetson/debug/message", EntryType.STRING),
        TIMESTAMP("jetson/debug/timestamp", EntryType.DOUBLE),

        OPPONENT_CAMERA("jetson/opponent_robot/camera", EntryType.STRING),
        OPPONENT_DISTANCE("jetson/opponent_robot/distance", EntryType.DOUBLE_ARRAY),
        OPPONENT_ANGLE("jetson/opponent_robot/angle", EntryType.DOUBLE_ARRAY),
        OPPONENT_TIMESTAMP("jetson/opponent_robot/timestamp", EntryType.DOUBLE_ARRAY),

        TEAMMATE_CAMERA("jetson/teammate_robot/camera", EntryType.STRING),
        TEAMMATE_DISTANCE("jetson/teammate_robot/distance", EntryType.DOUBLE_ARRAY),
        TEAMMATE_ANGLE("jetson/teammate_robot/angle", EntryType.DOUBLE_ARRAY),
        TEAMMATE_TIMESTAMP("jetson/teammate_robot/timestamp", EntryType.DOUBLE_ARRAY),

        // Algae
        ALGAE_CAMERA("jetson/algae/camera", EntryType.STRING),
        ALGAE_DISTANCE("jetson/algae/x", EntryType.DOUBLE_ARRAY),
        ALGAE_ANGLE("jetson/algae/angle", EntryType.DOUBLE_ARRAY),
        ALGAE_TIMESTAMP("jetson/algae/timestamp", EntryType.DOUBLE_ARRAY),

        // Coral
        CORAL_CAMERA("jetson/coral/camera", EntryType.STRING),
        CORAL_DISTANCE("jetson/coral/distace", EntryType.DOUBLE_ARRAY),
        CORAL_ANGLE("jetson/coral/angle", EntryType.DOUBLE_ARRAY),
        CORAL_TIMESTAMP("jetson/coral/timestamp", EntryType.DOUBLE_ARRAY);

        public enum EntryType {
            BOOLEAN,
            DOUBLE,
            INTEGER,
            STRING,
            DOUBLE_ARRAY
        }

        private final String key;
        private final EntryType type;

        JetsonToRio(String key, EntryType type) {
            this.key = key;
            this.type = type;
        }

        public String key() {
            return key;
        }

        public EntryType type() {
            return type;
        }
    }
}
