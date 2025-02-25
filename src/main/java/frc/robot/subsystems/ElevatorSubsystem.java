// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class ElevatorSubsystem extends SubsystemBase{
  private SparkMax elevatorMotor = new SparkMax(ClimbConstants.LEFT_CAN_ID, MotorType.kBrushless);

  public void elevatorUp() {
    elevatorMotor.set(ClimbConstants.CLIMB_SPEED);
  }

  public void elevatorDown() {
    elevatorMotor.set(ClimbConstants.CLIMB_SPEED);
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  @Override
  public void periodic() {}
}
