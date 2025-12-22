package frc.robot.constants.connection;

public class NetworkConstants {
    public static final String ROBOT_IP_ADDRESS = "10.37.56.2";
    public static final String NETWORK_TABLE_NAME = "AIPipeline";
    public static final String DATA_ENTRY_NAME = "data";

    public enum RioToJetson {
        ROBOT_ENABLED("robot/mode/enabled"),
        ROBOT_DISABLED("robot/mode/disabled"),
        ROBOT_ESTOP("robot/mode/estop"),
        ROBOT_AUTO("robot/mode/auto"),
        ROBOT_TELEOP("robot/mode/teleop"),
        ROBOT_TEST("robot/mode/test"),
        ROBOT_ALLIANCE("robot/alliance"),
        ROBOT_MATCH_TIME("robot/match_time"),
        ROBOT_START_LOCATION("robot/start_location"),
        ROBOT_POSE_X("robot/pose/x"),
        ROBOT_POSE_Y("robot/pose/y"),
        ROBOT_POSE_ROT("robot/pose/theta"),
        ROBOT_VELOCITY_X("robot/velocity/vx"),
        ROBOT_VELOCITY_Y("robot/velocity/vy"),
        ROBOT_VELOCITY_ROTATIONS("robot/velocity/vomega"),
        USER_SELECTED_AUTO("ds/auto/selection");

        private final String key;

        RioToJetson(String key) {
            this.key = key;
        }

        public String key() {
            return key;
        }
    }

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
        OPPONENT_X("jetson/opponent_robot/x", EntryType.DOUBLE_ARRAY),
        OPPONENT_Y("jetson/opponent_robot/y", EntryType.DOUBLE_ARRAY),
        OPPONENT_TIMESTAMP("jetson/opponent_robot/timestamp", EntryType.DOUBLE_ARRAY),

        TEAMMATE_CAMERA("jetson/teammate_robot/camera", EntryType.STRING),
        TEAMMATE_X("jetson/teammate_robot/x", EntryType.DOUBLE_ARRAY),
        TEAMMATE_Y("jetson/teammate_robot/y", EntryType.DOUBLE_ARRAY),
        TEAMMATE_TIMESTAMP("jetson/teammate_robot/timestamp", EntryType.DOUBLE_ARRAY),

        // Algae
        ALGAE_CAMERA("jetson/algae/camera", EntryType.STRING),
        ALGAE_X("jetson/algae/x", EntryType.DOUBLE_ARRAY),
        ALGAE_Y("jetson/algae/y", EntryType.DOUBLE_ARRAY),
        ALGAE_TIMESTAMP("jetson/algae/timestamp", EntryType.DOUBLE_ARRAY),

        // Coral
        CORAL_CAMERA("jetson/coral/camera", EntryType.STRING),
        CORAL_X("jetson/coral/x", EntryType.DOUBLE_ARRAY),
        CORAL_Y("jetson/coral/y", EntryType.DOUBLE_ARRAY),
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
