// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.constants.subsystems;

public class VisionConstants {
    public static final double MINIMUM_DRIVE_SPEED = 0.1;
    public static final double MAX_DRIVE_SPEED = 0.3;                           // 1 is very fast

    public static final double COMMAND_EXPIRATION_TIME = 0.25;                  // in seconds
    public static final double ROBOT_EXPONENTIAL_RAMP_UP_SPEED = 2;             // in seconds - 1 to 3
    
    public static final double DISTANCE_SCALING_FACTOR = 0.35;
    public static final double ANGULAR_TRUST_FACTOR = 10;
    public static final double MAX_XY_STD_DEV = 2.5;                            // in metres
	public static final double MIN_XY_STD_DEV = 0.15;                           // in metres
	public static final double MAX_THETA_STD_DEV = 30.0;                        // in degrees

    public static final double LINE_DEFENSE_PUSH_RESPONSE_DISTANCE = 0.6;
    public static final double LINE_DEFENSE_AGGRESSION_GAIN = 2.0;

	public static final String LIMELIGHT_3G_NAME = "limelight3g";
	public static final String LIMELIGHT_3_NAME = "limelight3";

    public static final double LIMELIGHT_3G_FORWARD = 0.0;
    public static final double LIMELIGHT_3G_RIGHT = 0.0;
    public static final double LIMELIGHT_3G_UP = 0.0;
    public static final double LIMELIGHT_3G_ROLL = 0.0;
    public static final double LIMELIGHT_3G_PITCH = 0.0;
    public static final double LIMELIGHT_3G_YAW = 0.0;

    public static final double LIMELIGHT_3_FORWARD = 0.0;
    public static final double LIMELIGHT_3_RIGHT = 0.0;
    public static final double LIMELIGHT_3_UP = 0.0;
    public static final double LIMELIGHT_3_ROLL = 0.0;
    public static final double LIMELIGHT_3_PITCH = 0.0;
    public static final double LIMELIGHT_3_YAW = 0.0;
}
