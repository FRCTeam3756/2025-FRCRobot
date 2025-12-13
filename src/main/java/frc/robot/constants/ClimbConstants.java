// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimbConstants {
    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
    public static final FeedbackSensor FEEDBACK_SENSOR = FeedbackSensor.kPrimaryEncoder;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static ResetMode RESET_MODE = ResetMode.kNoResetSafeParameters;
    public static PersistMode PERSIST_MODE = PersistMode.kPersistParameters;

    public static final int ERROR_CHECK_TIMEOUT = 20; // ms
    public static final double RAMP_RATE = 0.0;
    public static final int MOTOR_MAX_AMPERAGE = 60;

    public static boolean LEFT_MOTOR_INVERTED = true;
    public static boolean RIGHT_MOTOR_INVERTED = true;

    public static double LEFT_SPEED_PERCENTAGE = 1.00;
    public static double RIGHT_SPEED_PERCENTAGE = 0.90;

    public static double CLIMB_SPEED = 1.0;
}