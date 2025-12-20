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

        MESSAGE("debug/message", EntryType.STRING),
        TIMESTAMP("debug/timestamp", EntryType.DOUBLE);

        public enum EntryType {
            BOOLEAN,
            DOUBLE,
            INTEGER,
            STRING
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
