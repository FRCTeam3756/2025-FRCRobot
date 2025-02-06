// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
  private VictorSPX leftMotor;
  private TalonSRX rightMotor;

  public CoralSubsystem() {
    leftMotor = new VictorSPX(CoralConstants.LEFT_MOTOR_ID);
    rightMotor = new TalonSRX(CoralConstants.RIGHT_MOTOR_ID);
  }

  public void intakeGamePiece() {
    leftMotor.set(ControlMode.PercentOutput, CoralConstants.INTAKE_SPEED);
    rightMotor.set(ControlMode.PercentOutput, CoralConstants.INTAKE_SPEED);
  }

  public void outtakeGamePiece() {
    leftMotor.set(ControlMode.PercentOutput, CoralConstants.OUTTAKE_SPEED);
    rightMotor.set(ControlMode.PercentOutput, CoralConstants.OUTTAKE_SPEED);
  }

  public void shootL1() {
    setMotorSpeeds(CoralConstants.L1_SHOOT_SPEED);
  }

  public void shootL2() {
    setMotorSpeeds(CoralConstants.L2_SHOOT_SPEED);
  }

  public void shootL3() {
    setMotorSpeeds(CoralConstants.L3_SHOOT_SPEED);
  }

  public void shootL4() {
    setMotorSpeeds(CoralConstants.L4_SHOOT_SPEED);
  }

  public void stop() {
    setMotorSpeeds(0.0);
  }

  private void setMotorSpeeds(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, -speed);
  }

  @Override
  public void periodic() {
    
  }
}