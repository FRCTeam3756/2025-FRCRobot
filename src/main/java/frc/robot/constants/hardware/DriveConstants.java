// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants.hardware;

import com.pathplanner.lib.config.PIDConstants;

public class DriveConstants {
    public static final PIDConstants PATHPLANNER_DRIVE_PID = new PIDConstants(3.0, 0.0, 0.0);
    public static final PIDConstants PATHPLANNER_STEER_PID = new PIDConstants(4.0, 0.0, 0.1);

    public static final double TURBO_DRIVE_MULTIPLIER = 1;
    public static final double STANDARD_DRIVE_MULTIPLIER = 0.5;
    public static final double SLOW_DRIVE_MULTIPLIER = 0.25;
    public static final double SLUG_DRIVE_MULTIPLIER = 0.10;
}
