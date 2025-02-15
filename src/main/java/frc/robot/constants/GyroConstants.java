// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroConstants {
    public static final int CAN_PORT = 0;
    public static final String CAN_BUS_NAME = "rio";
    public static final String CAN_LOG_PATH = "./logs/canbus.hoot";
    public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(90);
    public static final Pigeon2Configuration PIGEON_CONFIG = null;
}