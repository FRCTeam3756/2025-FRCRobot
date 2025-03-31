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
  private final TalonSRX leftPaddle, rightPaddle;

  public ClimbingSubsystem() {
    leftPaddle = new TalonSRX(CANConstants.LEFT_CLIMB_MOTOR_ID);
    rightPaddle = new TalonSRX(CANConstants.RIGHT_CLIMB_MOTOR_ID);

    leftPaddle.setInverted(ClimbConstants.LEFT_MOTOR_INVERTED);
    rightPaddle.setInverted(ClimbConstants.RIGHT_MOTOR_INVERTED);

    leftPaddle.configOpenloopRamp(ClimbConstants.RAMP_RATE, ClimbConstants.ERROR_CHECK_TIMEOUT);
    leftPaddle.configOpenloopRamp(ClimbConstants.RAMP_RATE, ClimbConstants.ERROR_CHECK_TIMEOUT);

    leftPaddle.configClosedloopRamp(ClimbConstants.RAMP_RATE, ClimbConstants.ERROR_CHECK_TIMEOUT);
    leftPaddle.configClosedloopRamp(ClimbConstants.RAMP_RATE, ClimbConstants.ERROR_CHECK_TIMEOUT);
  }

  @Override
  public void periodic() {}

  public void climbing() {
    leftPaddle.set(ControlMode.PercentOutput, ClimbConstants.CLIMB_SPEED * ClimbConstants.LEFT_SPEED_PERCENTAGE);
    rightPaddle.set(ControlMode.PercentOutput, ClimbConstants.CLIMB_SPEED * ClimbConstants.RIGHT_SPEED_PERCENTAGE);
  }

  public void stopClimbing() {
    leftPaddle.set(ControlMode.Velocity, 0);
    rightPaddle.set(ControlMode.Velocity, 0);
  }
}