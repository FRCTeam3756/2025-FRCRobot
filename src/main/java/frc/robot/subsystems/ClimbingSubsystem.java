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

  private static final double STALL_CURRENT_THRESHOLD = -40.0; // 775's burn out at 50A
  private static final double STALL_TIME_THRESHOLD = 0.25;
  private static final double RECOVERY_TIME = 2.0;

  private double leftStallStartTime = -1;
  private double rightStallStartTime = -1;

  private boolean leftStalled = false;
  private boolean rightStalled = false;

  @Override
  public void periodic() {
    checkMotorStall();
  }

  public void climbing() {
    if (!leftStalled && !rightStalled) {
      leftPaddle.set(ControlMode.PercentOutput,
          ClimbConstants.LEFT_MOTOR_INVERSION * (ClimbConstants.CLIMB_SPEED * ClimbConstants.LEFT_SPEED_PERCENTAGE));
      rightPaddle.set(ControlMode.PercentOutput,
          ClimbConstants.RIGHT_MOTOR_INVERSION * (ClimbConstants.CLIMB_SPEED * ClimbConstants.RIGHT_SPEED_PERCENTAGE));
    }
  }

  public void stopClimbing() {
    leftPaddle.set(ControlMode.PercentOutput, 0);
    rightPaddle.set(ControlMode.PercentOutput, 0);
  }

  private void checkMotorStall() {
    double currentTime = Timer.getFPGATimestamp();

    leftStalled = updateStallStatus(leftPaddle, currentTime, leftStalled, leftStallStartTime);
    rightStalled = updateStallStatus(rightPaddle, currentTime, rightStalled, rightStallStartTime);
  }

  private boolean updateStallStatus(TalonSRX paddle, double currentTime, boolean motorStalled, double motorStallStartTime) {
    double motorCurrent = paddle.getSupplyCurrent();

    if (motorCurrent >= STALL_CURRENT_THRESHOLD) {
      if (motorStalled && currentTime - motorStallStartTime >= RECOVERY_TIME) {
        return false;
      }
      return motorStalled;
    } else {
      if (motorStallStartTime == -1) {
        motorStallStartTime = currentTime;
      } else if (currentTime - motorStallStartTime >= STALL_TIME_THRESHOLD) {
        stopClimbing();
        return true;
      }
    }

    return motorStalled;
  }
}