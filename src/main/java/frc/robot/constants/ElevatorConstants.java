// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorConstants {
    public static MotorType MOTOR_TYPE = MotorType.kBrushless;
    public static final FeedbackSensor FEEDBACK_SENSOR = FeedbackSensor.kPrimaryEncoder;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static ResetMode RESET_MODE = ResetMode.kNoResetSafeParameters;
    public static PersistMode PERSIST_MODE = PersistMode.kPersistParameters;

    public static final int MOTOR_MAX_AMPERAGE = 60;
    public static final double MOTOR_RAMP_RATE = 0.1;
    public static final double ELEVATOR_SPEED = 1.0;

    public static final double MINIMUM_OUTPUT = -1;
    public static final double MAXIMUM_OUTPUT = 1;

    public static final double P = 0.10;
    public static final double I = 0;
    public static final double D = 0.002;
    public static final double FF = 0.2;

    public static final double MAX_HEIGHT = 10000; // 630000
    public static final double MIN_HEIGHT = 0;

    public static final double ELEVATOR_L1_HEIGHT = 100;
    public static final double ELEVATOR_BOTTOM_ALGAE_HEIGHT = 4000;
    public static final double ELEVATOR_TOP_ALGAE_HEIGHT = 7000;
}