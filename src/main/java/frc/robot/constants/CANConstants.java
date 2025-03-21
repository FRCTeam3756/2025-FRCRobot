package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CANConstants {
    // 1 - 12 = Krakens, Falcons, CANCoders

    public static final int PIGEON_ID = 13;

    public static final int LEFT_CLIMB_MOTOR_ID = 14;
    public static final int RIGHT_CLIMB_MOTOR_ID = 15;
    
    public static final int ELEVATOR_MOTOR_ID = 16;

    public static final int WRIST_MOTOR_ID = 17;
    public static final int LEFT_CLAW_MOTOR_ID = 18;
    public static final int RIGHT_CLAW_MOTOR_ID = 19;
    
    public static final String CAN_BUS_NAME = "rio";
    public static final String CAN_LOG_PATH = "./logs/canbus.hoot";

    public static final SparkMax elevatorMotor = new SparkMax(CANConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    public static final SparkMax wristMotor = new SparkMax(CANConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    public static final SparkMax leftMotor = new SparkMax(CANConstants.LEFT_CLAW_MOTOR_ID, MotorType.kBrushless);
    public static final SparkMax rightMotor = new SparkMax(CANConstants.RIGHT_CLAW_MOTOR_ID, MotorType.kBrushless);
    
    public static final TalonSRX leftPaddle = new TalonSRX(CANConstants.LEFT_CLIMB_MOTOR_ID);
    public static final TalonSRX rightPaddle = new TalonSRX(CANConstants.RIGHT_CLIMB_MOTOR_ID);
}
