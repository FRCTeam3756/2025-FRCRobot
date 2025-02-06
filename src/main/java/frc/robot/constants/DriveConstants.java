// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  public static final NeutralModeValue NEUTRAL_DRIVE_MODE = NeutralModeValue.Coast;
  public static final IdleMode NEUTRAL_STEER_MODE = IdleMode.kCoast;

  public static final double DRIVE_SPEED_LIMIT = 1.0;
  public static final double ROTATION_SPEED_LIMIT = 1.0;

  public static final double MAX_MODULE_VELOCITY = 1000; //4.5
  public static final double MAX_ROBOT_VELOCITY = 1000; //4.5
  public static final double MAX_ROBOT_RADIAN_VELOCITY = 1000; //8.0

  public static final double FRAME_WIDTH = Units.inchesToMeters(28);
  public static final double FRAME_DEPTH = Units.inchesToMeters(28);

  public static final class ModuleIndices {
    public static final int FRONT_LEFT = 0;
    public static final int REAR_LEFT = 1;
    public static final int FRONT_RIGHT = 2;
    public static final int REAR_RIGHT = 3;
  }

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(FRAME_WIDTH / 2, FRAME_DEPTH / 2.),
      new Translation2d(FRAME_WIDTH / 2, -FRAME_DEPTH / 2),
      new Translation2d(-FRAME_WIDTH / 2, FRAME_DEPTH / 2),
      new Translation2d(-FRAME_WIDTH / 2, -FRAME_DEPTH / 2));
}
