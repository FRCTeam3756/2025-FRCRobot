// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.constants;

public class JetsonConstants {

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
        ROBOT_VELOCITY_ANGLE("robot/velocity/vtheta");

        private final String key;

        RioToJetson(String key) {
            this.key = key;
        }

        public String key() {
            return key;
        }
    }

    public enum JetsonToRio {
        DESIRED_VX("jetson/command/vx"),
        DESIRED_VY("jetson/command/vy"),
        DESIRED_VTHETA("jetson/command/vtheta"),
        DESIRED_TURBO_STATE("jetson/command/turbo"),

        DESIRED_INTAKE ("jetson/command/intake"),
        DESIRED_OUTTAKE("jetson/command/outtake"),
        DESIRED_SHOOT("jetson/command/shoot"),
        DESIRED_SHOOT_SPEED("jetson/command/shoot_rpm"),

        DESIRED_ELEVATOR_MODE("jetson/command/elevator_mode"),
        DESIRED_ELEVATOR_SETPOINT("jetson/command/elevator_setpoint"),

        DESIRED_WRIST_MODE("jetson/command/wrist_mode"),
        DESIRED_WRIST_ANGLE("jetson/command/wrist_angle"),

        DESIRED_ARM_MODE("jetson/command/arm_mode"),
        DESIRED_ARM_SETPOINT("jetson/command/arm_setpoint"),

        DESIRED_CLIMB("jetson/command/climb"),
        DESIRED_CLIMB_SETPOINT("jetson/command/climb_setpoint"),

        PATH_ACTIVE("jetson/path/active"),
        PATH_NAME("jetson/path/name"),
        PATH_PROGRESS("jetson/path/progress"),
        PATH_COMPLETE("jetson/path/complete"),

        AUTO_OBJECTIVE("jetson/strategy/objective"),
        AUTO_SUBTARGET("jetson/strategy/subtarget"),
        AUTO_PRIORITY("jetson/strategy/priority"),

        MESSAGE("debug/message"),
        TIMESTAMP("debug/timestamp");

        private final String key;

        JetsonToRio(String key) {
            this.key = key;
        }

        public String key() {
            return key;
        }
    }

    public static final double MINIMUM_DRIVE_SPEED = 0.1;
    public static final double MAX_DRIVE_SPEED = 0.3;                   // 1 is very fast

    public static final double COMMAND_EXPIRATION_TIME = 0.25;          // in seconds
    public static final double ROBOT_EXPONENTIAL_RAMP_UP_SPEED = 2;     // in seconds - 1 to 3

    public static final int ENABLE_INTAKE_DISTANCE = 50;                // in centimetres
    public static final int MAX_SPEED_DISTANCE = 500;                   // in centimetres
    public static final int MIN_SPEED_DISTANCE = 200;                   // in centimetres
}
