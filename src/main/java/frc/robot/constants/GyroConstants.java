// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroConstants {
    public static final int GYRO_CAN_PORT = 0;
    public static final String GYRO_CAN_BUS_NAME = "rio";
    public static final Rotation2d GYRO_ANGLE_OFFSET = Rotation2d.fromDegrees(90);
    public static final Pigeon2Configuration GYRO_CONFIG = null;
}