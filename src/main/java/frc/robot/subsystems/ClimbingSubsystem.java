// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.constants.ClimbConstants;
import frc.robot.constants.CANConstants;

public class ClimbingSubsystem extends SubsystemBase {
  public static final TalonSRX leftPaddle = new TalonSRX(CANConstants.LEFT_CLIMB_MOTOR_ID);
  public static final TalonSRX rightPaddle = new TalonSRX(CANConstants.RIGHT_CLIMB_MOTOR_ID);

  @Override
  public void periodic() {}

  public void climbing() {
    leftPaddle.set(ControlMode.PercentOutput,
        ClimbConstants.LEFT_MOTOR_INVERSION * (ClimbConstants.CLIMB_SPEED * ClimbConstants.LEFT_SPEED_PERCENTAGE));
    rightPaddle.set(ControlMode.PercentOutput,
        ClimbConstants.RIGHT_MOTOR_INVERSION * (ClimbConstants.CLIMB_SPEED * ClimbConstants.RIGHT_SPEED_PERCENTAGE));
  }

  public void stopClimbing() {
    leftPaddle.set(ControlMode.PercentOutput, 0);
    rightPaddle.set(ControlMode.PercentOutput, 0);
  }
}