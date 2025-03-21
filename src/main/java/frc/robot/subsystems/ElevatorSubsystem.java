// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.CANConstants;

public class ElevatorSubsystem extends SubsystemBase {
  public void elevatorUp() {
    CANConstants.elevatorMotor.set(ElevatorConstants.ELEVATOR_SPEED);
  }

  public void elevatorDown() {
    CANConstants.elevatorMotor.set(-ElevatorConstants.ELEVATOR_SPEED);
  }

  public void elevatorStop() {
    CANConstants.elevatorMotor.set(0);
  }

  @Override
  public void periodic() {}
}
