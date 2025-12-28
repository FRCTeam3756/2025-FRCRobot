// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.constants.subsystems;

public class VisionConstants {
    public static final double MINIMUM_DRIVE_SPEED = 0.1;
    public static final double MAX_DRIVE_SPEED = 0.3;                                   // 1 is full speed
    public static final double MAX_ROTATIONAL_RATE = Math.toRadians(180);               // 360 is one turn per second

    public static final double ALGAE_SELECTION_PROXIMITY_WEIGHT = 0.7;
    public static final double ALGAE_SELECTION_VELOCITY_WEIGHT = 0.3;

    public static final double ALGAE_INTAKE_DISTANCE = 1.0;                             // in metres
    public static final double ALGAE_FINE_TUNE_DISTANCE = 0.8;                          // in metres
    public static final double ALGAE_FINE_TUNE_FORWARD_GAIN = 1.0;                      // in m/s
    public static final double ALGAE_FINE_TUNE_ROTATIONAL_GAIN = Math.toRadians(30);    // in rad/s

    public static final double LINE_DEFENSE_PUSH_RESPONSE_DISTANCE = 0.6;
    public static final double LINE_DEFENSE_AGGRESSION_GAIN = 2.0;
}
