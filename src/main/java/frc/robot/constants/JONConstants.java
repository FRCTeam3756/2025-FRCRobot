// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.constants;

public class JONConstants {

    public static final String ROBOT_IP_ADDRESS = "10.37.56.2";
    public static final String NETWORK_TABLE_NAME = "AIPipeline";
    public static final String DATA_ENTRY_NAME = "data";

    public enum NETWORK_TABLE_NAMING {
        ROBOT_ENABLED("robot/mode/enabled"), // boolean
        ROBOT_DISABLED("robot/mode/disabled"), // boolean
        ROBOT_AUTO("robot/mode/auto"), // boolean
        ROBOT_TELEOP("robot/mode/teleop"), // boolean
        ROBOT_TEST("robot/mode/test"), // boolean
        ROBOT_ESTOP("robot/mode/estop"), // boolean

        ROBOT_ALLIANCE("robot/alliance"), // string: "red" or "blue"
        ROBOT_LOCATION("robot/location"), // int: station number
        ROBOT_MATCH_TIME("robot/match_time"), // double seconds remaining

        // Robot pose and motion
        ROBOT_POSE_X("robot/pose/x"), // meters
        ROBOT_POSE_Y("robot/pose/y"), // meters
        ROBOT_POSE_ROT("robot/pose/heading_deg"), // degrees
        ROBOT_VELOCITY_X("robot/velocity/vx"), // m/s
        ROBOT_VELOCITY_Y("robot/velocity/vy"), // m/s
        ROBOT_ANGULAR_VEL("robot/velocity/omega"), // rad/s

        // Sensors
        ROBOT_GYRO("robot/gyro/heading_deg"), // degrees
        ROBOT_ACCEL_X("robot/accel/x"), // m/s²
        ROBOT_ACCEL_Y("robot/accel/y"), // m/s²
        ROBOT_ACCEL_Z("robot/accel/z"), // m/s²
        ROBOT_CURRENT_DRAW("robot/power/current"), // amps

        // ---------------------------
        // JETSON → ROBOT (Drivetrain)
        // ---------------------------

        CMD_VX("jetson/cmd/vx"), // double m/s
        CMD_VY("jetson/cmd/vy"), // double m/s
        CMD_OMEGA("jetson/cmd/omega"), // double rad/s
        CMD_TURBO("jetson/cmd/turbo"), // boolean

        CMD_BRAKE("jetson/cmd/brake"), // boolean
        CMD_DRIVE_MODE("jetson/cmd/drive_mode"), // string ("field", "robot", "align")

        CMD_INTAKE("jetson/command/intake"), // boolean
        CMD_OUTTAKE("jetson/command/outtake"), // boolean
        CMD_SHOOT("jetson/command/shoot"), // boolean
        CMD_SHOOT_SPEED("jetson/command/shoot_rpm"), // double target RPM

        CMD_ELEVATOR_MODE("jetson/command/elevator_mode"), // string: "IDLE", "L1", "L2", etc.
        CMD_ELEVATOR_SETPOINT("jetson/command/elevator_setpoint"), // double meters

        CMD_WRIST_MODE("jetson/command/wrist_mode"), // string
        CMD_WRIST_ANGLE("jetson/command/wrist_angle"), // double degrees

        CMD_ARM_MODE("jetson/command/arm_mode"), // string
        CMD_ARM_SETPOINT("jetson/command/arm_setpoint"), // double degrees

        CMD_CLIMB("jetson/command/climb"), // boolean
        CMD_CLIMB_SETPOINT("jetson/command/climb_setpoint"),// double

        PATH_ACTIVE("jetson/path/active"), // boolean
        PATH_NAME("jetson/path/name"), // string
        PATH_PROGRESS("jetson/path/progress"), // 0.0–1.0
        PATH_COMPLETE("jetson/path/complete"), // boolean

        AUTO_OBJECTIVE("jetson/strategy/objective"), // string
        AUTO_SUBTARGET("jetson/strategy/subtarget"), // string
        AUTO_PRIORITY("jetson/strategy/priority"), // int

        MESSAGE("debug/message"), // string
        TIMESTAMP("debug/timestamp"); // double

        private final String value;

        NETWORK_TABLE_NAMING(String value) {
            this.value = value;
        }

        public String getValue() {
            return value;
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
