// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants;

public class JONConstants {
    public static final String ROBOT_IP_ADDRESS = "10.37.56.2";
    public static final String NETWORK_TABLE_NAME = "AIPipeline";
    public static final String DATA_ENTRY_NAME = "data";

    public enum NETWORK_TABLE_NAMING {
        X("x"),
        Y("y"),
        ROTATION("rot"),
        TURBO("turbo"),
        INTAKE("intake"),
        OUTTAKE("outtake"),
        ELEVATOR("elevator"),
        MATCH_PHASE("match_phase"),
        GOAL("goal"),
        GOAL_PICKUP_ALGAE("pickup_algae"),
        GOAL_SCORE_ALGAE("score_algae"),;

        private final String value;

        NETWORK_TABLE_NAMING(String value) {
            this.value = value;
        }

        public String getValue() {
            return value;
        }
    }

    public static final double MINIMUM_DRIVE_SPEED = 0.1;
    public static final double MAX_DRIVE_SPEED = 0.3; // 1 is very fast

    public static final double COMMAND_EXPIRATION_TIME = 0.25; // in seconds
    public static final double ROBOT_EXPONENTIAL_RAMP_UP_SPEED = 2; // in seconds - 1 to 3

    public static final int ENABLE_INTAKE_DISTANCE = 50; // in centimetres
    public static final int MAX_SPEED_DISTANCE = 500; // in centimetres
    public static final int MIN_SPEED_DISTANCE = 200; // in centimetres
}
