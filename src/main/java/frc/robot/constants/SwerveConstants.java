// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveConstants {
    public static final double STEER_MODULE_KP = 100; // Proportional Gain
    public static final double STEER_MODULE_KI = 0.0; // Integral Gain
    public static final double STEER_MODULE_KD = 0.5;  // Derivative Gain
    public static final double STEER_MODULE_KS = 0.1; // Proportional Gain
    public static final double STEER_MODULE_KV = 2.66; // Integral Gain
    public static final double STEER_MODULE_KA = 0.0;  // Derivative Gain

    public static final double DRIVE_MODULE_KP = 0.1; // Proportional Gain
    public static final double DRIVE_MODULE_KI = 0.0; // Integral Gain
    public static final double DRIVE_MODULE_KD = 0.0;  // Derivative Gain
    public static final double DRIVE_MODULE_KS = 0.0; // Static Gain
    public static final double DRIVE_MODULE_KV = 0.124; // Velocity Gain
    public static final double DRIVE_MODULE_KA = 0.0;  // Acceleration Gain

    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(STEER_MODULE_KP).withKI(STEER_MODULE_KI).withKD(STEER_MODULE_KD)
        .withKS(STEER_MODULE_KS).withKV(STEER_MODULE_KV).withKA(STEER_MODULE_KA);
        
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(DRIVE_MODULE_KP).withKI(DRIVE_MODULE_KI).withKD(DRIVE_MODULE_KD)
        .withKS(DRIVE_MODULE_KS).withKV(DRIVE_MODULE_KV).withKA(DRIVE_MODULE_KA);
        
    public static final double TURBO_DRIVE_MULTIPLIER = 0.5;
    public static final double STANDARD_DRIVE_MULTIPLIER = 0.25;

    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);
    public static final double MAX_SPEED = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1).in(RadiansPerSecond);

    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;
    private static final Current kSlipCurrent = Amps.of(120.0);
    
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

    private static final double COUPLE_RATIO = 3.5714285714285716;
    private static final double DRIVE_GEAR_RATIO = 6.746031746031747;
    private static final double STEER_GEAR_RATIO = 21.428571428571427;

    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean INVERT_LEFT = false;
    private static final boolean INVERT_RIGHT = true;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(GyroConstants.GYRO_CAN_BUS_NAME)
            .withPigeon2Id(GyroConstants.GYRO_CAN_PORT);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(DRIVE_MOTOR_TYPE)
            .withSteerMotorType(STEER_MOTOR_TYPE)
            .withFeedbackSource(STEER_FEEDBACK_TYPE)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs);


    // Front Left
    private static final int FL_DRIVE_ID = 1;
    private static final int FL_STEER_ID = 5;
    private static final int FL_CANCODER_ID = 9;
    private static final Angle FL_OFFSET_RADIANS = Rotations.of(-0.15234375);
    private static final boolean FL_STEER_MOTOR_REVERSED = false;
    private static final boolean FL_CANCODER_REVERSED = true;

    private static final Distance FL_X_POSITION = Inches.of(11.5);
    private static final Distance FL_Y_POSITION = Inches.of(11.5);

    // Front Right
    private static final int FR_DRIVE_ID = 2;
    private static final int FR_STEER_ID = 6;
    private static final int FR_CANCODER_ID = 10;
    private static final Angle FR_OFFSET_RADIANS = Rotations.of(0.282958984375);
    private static final boolean FR_STEER_MOTOR_REVERSED = false;
    private static final boolean FR_CANCODER_REVERSED = true;

    private static final Distance FR_X_POSITION = Inches.of(11.5);
    private static final Distance FR_Y_POSITION = Inches.of(-11.5);

    // Back Left
    private static final int BL_DRIVE_ID = 3;
    private static final int BL_STEER_ID = 7;
    private static final int BL_CANCODER_ID = 11;
    private static final Angle BL_OFFSET_RADIANS = Rotations.of(-0.18017578125);
    private static final boolean BL_STEER_MOTOR_REVERSED = false;
    private static final boolean BL_CANCODER_REVERSED = true;

    private static final Distance BL_X_POSITION = Inches.of(-11.5);
    private static final Distance BL_Y_POSITION = Inches.of(11.5);

    // Back Right
    private static final int BR_DRIVE_ID = 4;
    private static final int BR_STEER_ID = 8;
    private static final int BR_CANCODER_ID = 12;
    private static final Angle BR_OFFSET_RADIANS = Rotations.of(0.021240234375);
    private static final boolean BR_STEER_MOTOR_REVERSED = false;
    private static final boolean BR_CANCODER_REVERSED = true;

    private static final Distance BR_X_POSITION = Inches.of(-11.5);
    private static final Distance BR_Y_POSITION = Inches.of(-11.5);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FL_SWERVE_MODULE =
        ConstantCreator.createModuleConstants(
            FL_STEER_ID, FL_DRIVE_ID, FL_CANCODER_ID, FL_OFFSET_RADIANS,
            FL_X_POSITION, FL_Y_POSITION, INVERT_LEFT, FL_STEER_MOTOR_REVERSED, FL_CANCODER_REVERSED
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FR_SWERVE_MODULE =
        ConstantCreator.createModuleConstants(
            FR_STEER_ID, FR_DRIVE_ID, FR_CANCODER_ID, FR_OFFSET_RADIANS,
            FR_X_POSITION, FR_Y_POSITION, INVERT_RIGHT, FR_STEER_MOTOR_REVERSED, FR_CANCODER_REVERSED
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BL_SWERVE_MODULE =
        ConstantCreator.createModuleConstants(
            BL_STEER_ID, BL_DRIVE_ID, BL_CANCODER_ID, BL_OFFSET_RADIANS,
            BL_X_POSITION, BL_Y_POSITION, INVERT_LEFT, BL_STEER_MOTOR_REVERSED, BL_CANCODER_REVERSED
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BR_SWERVE_MODULE =
        ConstantCreator.createModuleConstants(
            BR_STEER_ID, BR_DRIVE_ID, BR_CANCODER_ID, BR_OFFSET_RADIANS,
            BR_X_POSITION, BR_Y_POSITION, INVERT_RIGHT, BR_STEER_MOTOR_REVERSED, BR_CANCODER_REVERSED
        );

    public static SwerveSubsystem createDrivetrain() {
        return new SwerveSubsystem(
            DrivetrainConstants, FL_SWERVE_MODULE, FR_SWERVE_MODULE, BL_SWERVE_MODULE, BR_SWERVE_MODULE
        );
    }

    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}