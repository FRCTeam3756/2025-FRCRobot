// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ClimbConstants;

public class ClimbingSubsystem extends SubsystemBase {
  private TalonSRX leftPaddle = new TalonSRX(ClimbConstants.LEFT_CAN_ID);
  private TalonSRX rightPaddle = new TalonSRX(ClimbConstants.RIGHT_CAN_ID);

  public void climbingUp() {
    leftPaddle.set(ControlMode.PercentOutput, ClimbConstants.CLIMB_SPEED);
    rightPaddle.set(ControlMode.PercentOutput, ClimbConstants.CLIMB_SPEED * ClimbConstants.RIGHT_ADDITIONAL_SPEED_PERCENTAGE);
  }

  public void climbingDown() {
    leftPaddle.set(ControlMode.PercentOutput, -ClimbConstants.CLIMB_SPEED);
    rightPaddle.set(ControlMode.PercentOutput, -ClimbConstants.CLIMB_SPEED * ClimbConstants.RIGHT_ADDITIONAL_SPEED_PERCENTAGE);
  }

  public void climbingStop() {
    leftPaddle.set(ControlMode.PercentOutput, 0);
    rightPaddle.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {}
}