// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.CANConstants;
import frc.robot.constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  public void tiltWristUp() {
    CANConstants.wristMotor.set(ClawConstants.WRIST_UP_SPEED);
  }

  public void tiltWristDown() {
    CANConstants.wristMotor.set(ClawConstants.WRIST_DOWN_SPEED);
  }

  public void tiltWristStop() {
    CANConstants.wristMotor.set(0.0);
  }
  
  public void autoTiltWrist(double speed) {
    CANConstants.wristMotor.set(speed);
  }

  public void intakeRollers() {
    setRollerMotorSpeeds(ClawConstants.INTAKE_SPEED);
  }

  public void outtakeRollers() {
    setRollerMotorSpeeds(ClawConstants.OUTTAKE_SPEED);
  }

  public void stopRollers() {
    setRollerMotorSpeeds(0.0);
  }

  private void setRollerMotorSpeeds(double speed) {
    CANConstants.leftMotor.set(speed);
    CANConstants.rightMotor.set(-speed);
  }

  @Override
  public void periodic() {
    
  }
}