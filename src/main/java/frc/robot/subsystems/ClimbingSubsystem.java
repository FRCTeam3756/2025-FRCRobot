// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class ClimbingSubsystem extends SubsystemBase {
  private final TalonSRX leftPaddle = new TalonSRX(ClimbConstants.LEFT_CAN_ID);
  private final TalonSRX rightPaddle = new TalonSRX(ClimbConstants.RIGHT_CAN_ID);

  public void climbing() {
    leftPaddle.set(ControlMode.PercentOutput, ClimbConstants.CLIMB_SPEED * ClimbConstants.LEFT_ADDITIONAL_SPEED_PERCENTAGE);
    rightPaddle.set(ControlMode.PercentOutput, -ClimbConstants.CLIMB_SPEED * ClimbConstants.RIGHT_ADDITIONAL_SPEED_PERCENTAGE);
  }

  public void stop() {
    leftPaddle.set(ControlMode.PercentOutput, 0);
    rightPaddle.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {}
}