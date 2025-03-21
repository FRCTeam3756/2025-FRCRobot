// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.CANConstants;
import frc.robot.constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  public void tiltWristStop() {
    CANConstants.wristMotor.set(0.0);
  }

  public void tiltWristUp() {
    CANConstants.wristMotor.set(ClawConstants.WRIST_UP_SPEED);
  }

  public void tiltWristDown() {
    CANConstants.wristMotor.set(ClawConstants.WRIST_DOWN_SPEED);
  }

  public void stopRollers() {
    CANConstants.leftMotor.set(0.0);
    CANConstants.rightMotor.set(0.0);
  }

  public void intakeGamePiece() {
    setMotorSpeeds(ClawConstants.INTAKE_SPEED);
  }

  public void outtakeGamePiece() {
    setMotorSpeeds(ClawConstants.OUTTAKE_SPEED);
  }

  public void shootProcessor() {
    setMotorSpeeds(ClawConstants.PROCESSOR_SHOOT_SPEED);
  }

  public void shootL1() {
    setMotorSpeeds(ClawConstants.L1_SHOOT_SPEED);
  }

  public void shootL2() {
    setMotorSpeeds(ClawConstants.L2_SHOOT_SPEED);
  }

  public void shootL3() {
    setMotorSpeeds(ClawConstants.L3_SHOOT_SPEED);
  }

  public void shootL4() {
    setMotorSpeeds(ClawConstants.L4_SHOOT_SPEED);
  }

  public void stop() {
    setMotorSpeeds(0.0);
  }

  private void setMotorSpeeds(double speed) {
    CANConstants.leftMotor.set(speed);
    CANConstants.rightMotor.set(-speed);
  }

  @Override
  public void periodic() {
    
  }
}