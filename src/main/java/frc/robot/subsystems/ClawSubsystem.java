// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.CANConstants;
import frc.robot.constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  private final SparkMax wristMotor = new SparkMax(CANConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax leftMotor = new SparkMax(CANConstants.LEFT_CLAW_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(CANConstants.RIGHT_CLAW_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder wristEncoder = wristMotor.getEncoder();

  public ClawSubsystem() {
    wristEncoder.setPosition(0);
  }

  @Override
  public void periodic() {}

  public void tiltWristUp() {
    if (wristEncoder.getPosition() < ClawConstants.WRIST_MAX_HEIGHT) {
      wristMotor.set(ClawConstants.WRIST_UP_SPEED);
    } else {
      tiltWristStop();
    }
  }

  public void tiltWristDown() {
    if (wristEncoder.getPosition() > ClawConstants.WRIST_MIN_HEIGHT) {
      wristMotor.set(ClawConstants.WRIST_UP_SPEED);
    } else {
      tiltWristStop();
    }
  }

  public void tiltWristStop() {
    wristMotor.set(0.0);
  }
  
  public void autoTiltWrist(double speed) {
    if ((speed > 0) && (wristEncoder.getPosition() < ClawConstants.WRIST_MAX_HEIGHT)) {
      wristMotor.set(speed);
    } else if ((speed < 0) && (wristEncoder.getPosition() > ClawConstants.WRIST_MIN_HEIGHT)) {
      wristMotor.set(speed);
    } else {
      tiltWristStop();
    }
  }

  public void intakeRollers() {
    setRollerMotorSpeeds(ClawConstants.INTAKE_SPEED);
  }

  public void outtakeRollers() {
    setRollerMotorSpeeds(ClawConstants.OUTTAKE_SPEED);
  }

  public void autoIntakeRollers(double power) {
    setRollerMotorSpeeds(power);
  }

  public void autoOuttakeRollers(double power) {
    setRollerMotorSpeeds(power);
  }

  public void stopRollers() {
    setRollerMotorSpeeds(0.0);
  }

  private void setRollerMotorSpeeds(double speed) {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }
}