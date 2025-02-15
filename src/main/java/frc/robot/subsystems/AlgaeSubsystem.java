// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
  private VictorSPX leftMotor;
  private TalonSRX rightMotor;

  public AlgaeSubsystem() {
    leftMotor = new VictorSPX(AlgaeConstants.LEFT_MOTOR_ID);
    rightMotor = new TalonSRX(AlgaeConstants.RIGHT_MOTOR_ID);
  }

  public void intakeGamePiece() {
    leftMotor.set(ControlMode.PercentOutput, AlgaeConstants.INTAKE_SPEED);
    rightMotor.set(ControlMode.PercentOutput, AlgaeConstants.INTAKE_SPEED);
  }

  public void outtakeGamePiece() {
    leftMotor.set(ControlMode.PercentOutput, AlgaeConstants.OUTTAKE_SPEED);
    rightMotor.set(ControlMode.PercentOutput, AlgaeConstants.OUTTAKE_SPEED);
  }

  public void shootBarge() {
    setMotorSpeeds(AlgaeConstants.BARGE_SHOOT_SPEED);
  }

  public void shootProcessor() {
    setMotorSpeeds(AlgaeConstants.PROCESSOR_SHOOT_SPEED);
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