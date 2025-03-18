// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax elevatorMotor;
  private final SparkMaxConfig motorConfig;
  private final RelativeEncoder elevatorEncoder;

  public ElevatorSubsystem() {
    elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_CAN_ID, MotorType.kBrushless);

    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(ElevatorConstants.MOTOR_MAX_AMPERAGE);

    elevatorEncoder = elevatorMotor.getEncoder();

    REVLibError error = elevatorMotor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    if (error == REVLibError.kOk) {
      elevatorEncoder.setPosition(0);
    } else {
      System.err.println("Elevator Motor Configuration Failed: " + error.toString());
    }
  }

  public void elevatorUp() {
    if (getElevatorPosition() > ElevatorConstants.MAX_HEIGHT) {
      elevatorStop();
    } else {
      elevatorMotor.set(ClimbConstants.CLIMB_SPEED);
    }
  }

  public void elevatorDown() {
    if (getElevatorPosition() < ElevatorConstants.MIN_HEIGHT) {
      elevatorStop();
    } else {
      elevatorMotor.set(-ClimbConstants.CLIMB_SPEED);
    }
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
  }
}
