// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.connection.PortConstants;
import frc.robot.constants.hardware.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final SparkMaxConfig elevatorConfig;
    private final SparkClosedLoopController elevatorPID;

    public ElevatorSubsystem() {
        elevatorMotor = new SparkMax(PortConstants.ELEVATOR_MOTOR_ID, ElevatorConstants.MOTOR_TYPE);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorConfig = new SparkMaxConfig();
        elevatorPID = elevatorMotor.getClosedLoopController();

        elevatorEncoder.setPosition(0.0);

        elevatorConfig
                .closedLoopRampRate(ElevatorConstants.MOTOR_RAMP_RATE)
                .idleMode(ElevatorConstants.IDLE_MODE)
                .smartCurrentLimit(ElevatorConstants.MOTOR_MAX_AMPERAGE)

                .closedLoop
                .feedbackSensor(ElevatorConstants.FEEDBACK_SENSOR)
                .p(ElevatorConstants.P)
                .i(ElevatorConstants.I)
                .d(ElevatorConstants.D)
                .velocityFF(ElevatorConstants.FF)
                .outputRange(
                    ElevatorConstants.MINIMUM_OUTPUT, 
                    ElevatorConstants.MAXIMUM_OUTPUT
                );

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

    public void elevatorToSetpoint(int setpoint) {
        double rotations = ElevatorConstants.ELEVATOR_SETPOINTS[setpoint];
        elevatorPID.setReference(rotations, ControlType.kPosition);
    }
}
