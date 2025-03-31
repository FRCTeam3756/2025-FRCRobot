// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants;

public class CANConstants {
    // 1 - 12 = Krakens, Falcons, CANCoders

    public static final int PIGEON_ID = 13;

    public static final int LEFT_CLIMB_MOTOR_ID = 14;
    public static final int RIGHT_CLIMB_MOTOR_ID = 15;
    
    public static final int ELEVATOR_MOTOR_ID = 16;

    public static final int WRIST_MOTOR_ID = 17;
    public static final int LEFT_CLAW_MOTOR_ID = 18;
    public static final int RIGHT_CLAW_MOTOR_ID = 19;
    
    public static final String CAN_BUS_NAME = "rio";
    public static final String CAN_LOG_PATH = "./logs/canbus.hoot";
}
