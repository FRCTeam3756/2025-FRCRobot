// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.Units;

public class SwerveConstants {
    public static final double STEER_MODULE_KP = 100;   // Proportional Gain
    public static final double STEER_MODULE_KI = 0.0;   // Integral Gain
    public static final double STEER_MODULE_KD = 0.5;   // Derivative Gain
    public static final double STEER_MODULE_KS = 0.1;   // Static Gain
    public static final double STEER_MODULE_KV = 2.66;  // Velocity Gain
    public static final double STEER_MODULE_KA = 0.0;   // Acceleration Gain

    public static final double DRIVE_MODULE_KP = 0.1;   // Proportional Gain
    public static final double DRIVE_MODULE_KI = 0.0;   // Integral Gain
    public static final double DRIVE_MODULE_KD = 0.0;   // Derivative Gain
    public static final double DRIVE_MODULE_KS = 0.0;   // Static Gain
    public static final double DRIVE_MODULE_KV = 0.124; // Velocity Gain
    public static final double DRIVE_MODULE_KA = 0.0;   // Acceleration Gain

    public static final double TURBO_DRIVE_MULTIPLIER = 1;
    public static final double STANDARD_DRIVE_MULTIPLIER = 0.5;
    public static final double SLOW_DRIVE_MULTIPLIER = 0.25;

    private static final Slot0Configs STEER_GJONNS = new Slot0Configs()
        .withKP(STEER_MODULE_KP).withKI(STEER_MODULE_KI).withKD(STEER_MODULE_KD)
        .withKS(STEER_MODULE_KS).withKV(STEER_MODULE_KV).withKA(STEER_MODULE_KA);
        
    private static final Slot0Configs DRIVE_GJONNS = new Slot0Configs()
        .withKP(DRIVE_MODULE_KP).withKI(DRIVE_MODULE_KI).withKD(DRIVE_MODULE_KD)
        .withKS(DRIVE_MODULE_KS).withKV(DRIVE_MODULE_KV).withKA(DRIVE_MODULE_KA);

    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    
    private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.RemoteCANcoder;
    private static final Current SLIP_CURRENT = Units.Amps.of(120.0);

    private static final TalonFXConfiguration DRIVE_INITIAL_CONFIG = new TalonFXConfiguration();
    private static final TalonFXConfiguration STEER_INITIAL_CONFIG = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(Units.Amps.of(60))
                            .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration ENCODER_INITIAL_CONFIG = new CANcoderConfiguration();

    public static final CANBus CAN_BUS = new CANBus(GyroConstants.CAN_BUS_NAME, GyroConstants.CAN_LOG_PATH);

    public static final LinearVelocity SPEED_AT_12_VOLTS = Units.MetersPerSecond.of(4.73);

    private static final double COUPLE_RATIO = 3.5714285714285716;
    private static final double DRIVE_GEAR_RATIO = 6.75;                // SDS MK4i L2 Gear Ratio
    private static final double STEER_GEAR_RATIO = 150 / 7;

    private static final Distance WHEEL_RADIUS = Units.Inches.of(2);

    private static final boolean INVERT_LEFT = false;
    private static final boolean INVERT_RIGHT = true;

    private static final MomentOfInertia STEER_INERTIA = Units.KilogramSquareMeters.of(0.004);
    private static final MomentOfInertia DRIVE_INERTIA = Units.KilogramSquareMeters.of(0.025);

    private static final Voltage STEER_FRICTION_VOLTAGE = Units.Volts.of(0.2);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Units.Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(GyroConstants.CAN_BUS_NAME)
            .withPigeon2Id(GyroConstants.CAN_ID)
            .withPigeon2Configs(GyroConstants.PIGEON_CONFIG);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> CONSTANT_CREATOR = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withWheelRadius(WHEEL_RADIUS)
            .withSteerMotorGains(STEER_GJONNS)
            .withDriveMotorGains(DRIVE_GJONNS)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSlipCurrent(SLIP_CURRENT)
            .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
            .withDriveMotorType(DRIVE_MOTOR_TYPE)
            .withSteerMotorType(STEER_MOTOR_TYPE)
            .withFeedbackSource(STEER_FEEDBACK_TYPE)
            .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIG)
            .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIG)
            .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIG)
            .withSteerInertia(STEER_INERTIA)
            .withDriveInertia(DRIVE_INERTIA)
            .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

    // Front Left
    private static final int FL_DRIVE_ID = 1;
    private static final int FL_STEER_ID = 5;
    private static final int FL_CANCODER_ID = 9;
    private static final Angle FL_ENCODER_OFFSET = Units.Rotations.of(0.174560546875);
    private static final boolean FL_STEER_MOTOR_INVERTED = true;
    private static final boolean FL_CANCODER_INVERTED = false;

    public static final Distance FL_X_POSITION = Units.Inches.of(11.5);
    public static final Distance FL_Y_POSITION = Units.Inches.of(11.5);

    // Front Right
    private static final int FR_DRIVE_ID = 2;
    private static final int FR_STEER_ID = 6;
    private static final int FR_CANCODER_ID = 10;
    private static final Angle FR_ENCODER_OFFSET = Units.Rotations.of(-0.28076171875);
    private static final boolean FR_STEER_MOTOR_INVERTED = true;
    private static final boolean FR_CANCODER_INVERTED = false;

    public static final Distance FR_X_POSITION = Units.Inches.of(11.5);
    public static final Distance FR_Y_POSITION = Units.Inches.of(-11.5);

    // Back Left
    private static final int BL_DRIVE_ID = 3;
    private static final int BL_STEER_ID = 7;
    private static final int BL_CANCODER_ID = 11;
    private static final Angle BL_ENCODER_OFFSET = Units.Rotations.of(0.19287109375);
    private static final boolean BL_STEER_MOTOR_INVERTED = true;
    private static final boolean BL_CANCODER_INVERTED = false;

    public static final Distance BL_X_POSITION = Units.Inches.of(-11.5);
    public static final Distance BL_Y_POSITION = Units.Inches.of(11.5);

    // Back Right
    private static final int BR_DRIVE_ID = 4;
    private static final int BR_STEER_ID = 8;
    private static final int BR_CANCODER_ID = 12;
    private static final Angle BR_ENCODER_OFFSET = Units.Rotations.of(-0.0302734375);
    private static final boolean BR_STEER_MOTOR_INVERTED = true;
    private static final boolean BR_CANCODER_INVERTED = false;

    public static final Distance BR_X_POSITION = Units.Inches.of(-11.5);
    public static final Distance BR_Y_POSITION = Units.Inches.of(-11.5);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FL_SWERVE_MODULE =
        CONSTANT_CREATOR.createModuleConstants(
            FL_STEER_ID, FL_DRIVE_ID, FL_CANCODER_ID, FL_ENCODER_OFFSET,
            FL_X_POSITION, FL_Y_POSITION, INVERT_LEFT, FL_STEER_MOTOR_INVERTED, FL_CANCODER_INVERTED
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FR_SWERVE_MODULE =
        CONSTANT_CREATOR.createModuleConstants(
            FR_STEER_ID, FR_DRIVE_ID, FR_CANCODER_ID, FR_ENCODER_OFFSET,
            FR_X_POSITION, FR_Y_POSITION, INVERT_RIGHT, FR_STEER_MOTOR_INVERTED, FR_CANCODER_INVERTED
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BL_SWERVE_MODULE =
        CONSTANT_CREATOR.createModuleConstants(
            BL_STEER_ID, BL_DRIVE_ID, BL_CANCODER_ID, BL_ENCODER_OFFSET,
            BL_X_POSITION, BL_Y_POSITION, INVERT_LEFT, BL_STEER_MOTOR_INVERTED, BL_CANCODER_INVERTED
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BR_SWERVE_MODULE =
        CONSTANT_CREATOR.createModuleConstants(
            BR_STEER_ID, BR_DRIVE_ID, BR_CANCODER_ID, BR_ENCODER_OFFSET,
            BR_X_POSITION, BR_Y_POSITION, INVERT_RIGHT, BR_STEER_MOTOR_INVERTED, BR_CANCODER_INVERTED
        );


    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        }

        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new,
                    TalonFX::new,
                    CANcoder::new,
                    drivetrainConstants,
                    odometryUpdateFrequency,
                    modules);
        }

        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new,
                    TalonFX::new,
                    CANcoder::new,
                    drivetrainConstants,
                    odometryUpdateFrequency,
                    odometryStandardDeviation,
                    visionStandardDeviation,
                    modules);
        }
    }
}
