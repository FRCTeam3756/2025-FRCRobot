// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClawConstants {
    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
    public static final FeedbackSensor FEEDBACK_SENSOR = FeedbackSensor.kPrimaryEncoder;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static ResetMode RESET_MODE = ResetMode.kNoResetSafeParameters;
    public static PersistMode PERSIST_MODE = PersistMode.kPersistParameters;
    
    public static final int MOTOR_MAX_AMPERAGE = 30;
    public static final double MOTOR_RAMP_RATE = 0.1;

    public static final double MINIMUM_OUTPUT = -1.0;
    public static final double MAXIMUM_OUTPUT = 1.0;

    public static final double P = 0.15;
    public static final double I = 0;
    public static final double D = 0.002;
    public static final double FF = 0.05;

    public static final double WRIST_UP_SPEED = 0.20;
    public static final double WRIST_DOWN_SPEED = -0.15;

    public static final double INTAKE_SPEED = 0.4;
    public static final double OUTTAKE_SPEED = -1.0;

    // public static final double WRIST_MIN_HEIGHT = -400;
    // public static final double WRIST_MAX_HEIGHT = 0;
}