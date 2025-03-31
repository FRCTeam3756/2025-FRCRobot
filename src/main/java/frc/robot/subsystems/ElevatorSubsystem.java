// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.CANConstants;

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
        .smartCurrentLimit(ElevatorConstants.MOTOR_MAX_AMPERAGE)
        .closedLoop
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
    SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getPosition());
  }
  
  public void elevatorUp() {
    // if (elevatorEncoder.getPosition() < ElevatorConstants.MAX_HEIGHT) {
      elevatorMotor.set(ElevatorConstants.ELEVATOR_SPEED);
    // } else {
    //   elevatorStop();
    // }
  }

  public void elevatorDown() {
    // if (elevatorEncoder.getPosition() > ElevatorConstants.MIN_HEIGHT) {
      elevatorMotor.set(-ElevatorConstants.ELEVATOR_SPEED);
    // } else {
    //   elevatorStop();
    // }
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  public void autoElevator(double speed) {
    // if ((speed > 0) && (elevatorEncoder.getPosition() < ElevatorConstants.MAX_HEIGHT)) {
      elevatorMotor.set(speed);
    // } else if ((speed < 0) && (elevatorEncoder.getPosition() > ElevatorConstants.MIN_HEIGHT)) {
    //   elevatorMotor.set(speed);
    // } else {
    //   elevatorStop();
    // }
  }
}
