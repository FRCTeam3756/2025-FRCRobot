// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final SparkMaxConfig elevatorConfig;

    public ElevatorSubsystem() {
        elevatorMotor = new SparkMax(CANConstants.ELEVATOR_MOTOR_ID, ElevatorConstants.MOTOR_TYPE);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorConfig = new SparkMaxConfig();

        elevatorEncoder.setPosition(0);

        elevatorConfig
                .closedLoopRampRate(ElevatorConstants.MOTOR_RAMP_RATE)
                .idleMode(ElevatorConstants.IDLE_MODE)
                .smartCurrentLimit(ElevatorConstants.MOTOR_MAX_AMPERAGE).closedLoop
                .feedbackSensor(ElevatorConstants.FEEDBACK_SENSOR)
                .p(ElevatorConstants.P)
                .i(ElevatorConstants.I)
                .d(ElevatorConstants.D)
                .velocityFF(ElevatorConstants.FF)
                .outputRange(ElevatorConstants.MINIMUM_OUTPUT, ElevatorConstants.MAXIMUM_OUTPUT);

        elevatorMotor.configure(elevatorConfig, ElevatorConstants.RESET_MODE, ElevatorConstants.PERSIST_MODE);
    }

    @Override
    public void periodic() {
    }

    public void elevatorUp() {
        elevatorMotor.set(ElevatorConstants.ELEVATOR_SPEED);
    }

    public void elevatorDown() {
        elevatorMotor.set(-ElevatorConstants.ELEVATOR_SPEED);
    }

    public void elevatorStop() {
        elevatorMotor.set(0);
    }

    public void autoElevator(double speed) {
        elevatorMotor.set(speed);
    }
}
