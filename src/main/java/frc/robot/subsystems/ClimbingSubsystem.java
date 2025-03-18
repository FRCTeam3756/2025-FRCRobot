// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.constants.ClimbConstants;

public class ClimbingSubsystem extends SubsystemBase {
  private final TalonSRX leftPaddle = new TalonSRX(ClimbConstants.LEFT_CAN_ID);
  private final TalonSRX rightPaddle = new TalonSRX(ClimbConstants.RIGHT_CAN_ID);

  private static final double STALL_CURRENT_THRESHOLD = 40.0; // 775's burn out at 50A
  private static final double STALL_TIME_THRESHOLD = 0.25;
  private static final double RECOVERY_TIME = 2.0;

  private double leftStallStartTime = 0;
  private double rightStallStartTime = 0;
  private boolean leftStalled = false;
  private boolean rightStalled = false;

  @Override
  public void periodic() {
    double leftCurrent = leftPaddle.getStatorCurrent();
    double rightCurrent = rightPaddle.getStatorCurrent();
    
    preventMotorBurnout(leftPaddle, leftCurrent, leftStalled, leftStallStartTime);
    preventMotorBurnout(rightPaddle, rightCurrent, rightStalled, rightStallStartTime);
  }

  public void climbing() {
    if (!leftStalled && !rightStalled) {
      leftPaddle.set(ControlMode.PercentOutput,
          ClimbConstants.LEFT_MOTOR_INVERSION * (ClimbConstants.CLIMB_SPEED * ClimbConstants.LEFT_SPEED_PERCENTAGE));
      rightPaddle.set(ControlMode.PercentOutput,
          ClimbConstants.RIGHT_MOTOR_INVERSION * (ClimbConstants.CLIMB_SPEED * ClimbConstants.RIGHT_SPEED_PERCENTAGE));
    }
  }

  public void stop() {
    leftPaddle.set(ControlMode.PercentOutput, 0);
    rightPaddle.set(ControlMode.PercentOutput, 0);
    leftStalled = false;
    rightStalled = false;
  }

  private static void preventMotorBurnout(TalonSRX paddle, double motorCurrent, boolean motorStalled, double motorStallStartTime) {
    double currentTime = Timer.getFPGATimestamp();

    if (motorCurrent > STALL_CURRENT_THRESHOLD) {
      if (motorStallStartTime == 0) {
        motorStallStartTime = currentTime;
      } else if (currentTime - motorStallStartTime >= STALL_TIME_THRESHOLD) {
        paddle.set(ControlMode.PercentOutput, 0);
        motorStalled = true;
        motorStallStartTime = currentTime;
      }
    } else if (motorStalled && (currentTime - motorStallStartTime >= RECOVERY_TIME)) {
      motorStalled = false;
    } else if (motorCurrent < STALL_CURRENT_THRESHOLD) {
      motorStallStartTime = 0;
    }
  }
}