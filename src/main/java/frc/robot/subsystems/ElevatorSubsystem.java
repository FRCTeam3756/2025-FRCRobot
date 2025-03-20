// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.CANConstants;

public class ElevatorSubsystem extends SubsystemBase{
  private final SparkMax elevatorMotor;
  private final SparkMaxConfig motorConfig;
  
  public ElevatorSubsystem() {
    elevatorMotor = new SparkMax(CANConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(ElevatorConstants.MOTOR_MAX_AMPERAGE);

    REVLibError error = elevatorMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    if (error != REVLibError.kOk) {
      System.err.println("Elevator Motor Configuration Failed: " + error.toString());
    }
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

  @Override
  public void periodic() {}
}
