// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.constants.subsystems;

public class VisionConstants {
    public static final double MINIMUM_DRIVE_SPEED = 0.1;
    public static final double MAX_DRIVE_SPEED = 0.3;                   // 1 is very fast

    public static final double COMMAND_EXPIRATION_TIME = 0.25;          // in seconds
    public static final double ROBOT_EXPONENTIAL_RAMP_UP_SPEED = 2;     // in seconds - 1 to 3

    public static final int ENABLE_INTAKE_DISTANCE = 50;                // in centimetres
    public static final int MAX_SPEED_DISTANCE = 500;                   // in centimetres
    public static final int MIN_SPEED_DISTANCE = 200;                   // in centimetres
    
    public static final double VISION_DISTANCE_SCALING_FACTOR = 0.35;
    public static final double MAX_VISION_STD_DEV = 2.5;
	public static final double MIN_VISION_STD_DEV = 0.15;

    public static final double LINE_DEFENSE_PUSH_RESPONSE_DISTANCE = 0.6;
    public static final double LINE_DEFENSE_AGGRESSION_GAIN = 2.0;

	public static final String LIMELIGHT_3G_NAME = "limelight3g";
	public static final String LIMELIGHT_3_NAME = "limelight3";
}
