// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.CANConstants;

public class ElevatorSubsystem extends SubsystemBase {
  public static final SparkMax elevatorMotor = new SparkMax(CANConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  public ElevatorSubsystem() {
    elevatorEncoder.setPosition(0);
  }
  
  public void elevatorUp() {
    if (elevatorEncoder.getPosition() < ElevatorConstants.MAX_HEIGHT) {
      elevatorMotor.set(ElevatorConstants.ELEVATOR_SPEED);
    } else {
      elevatorStop();
    }
  }

  public void elevatorDown() {
    if (elevatorEncoder.getPosition() > ElevatorConstants.MIN_HEIGHT) {
      elevatorMotor.set(-ElevatorConstants.ELEVATOR_SPEED);
    } else {
      elevatorStop();
    }
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  public void autoElevator(double speed) {
    if (speed > 0 && elevatorEncoder.getPosition() < ElevatorConstants.MAX_HEIGHT) {
      elevatorMotor.set(speed);
    } else if (speed < 0 && elevatorEncoder.getPosition() > ElevatorConstants.MIN_HEIGHT) {
      elevatorMotor.set(speed);
    } else {
      elevatorStop();
    }
  }

  @Override
  public void periodic() {}
}
