// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  private final SparkMax wristMotor, leftMotor, rightMotor;
  private final SparkMaxConfig neoConfig;
  private final RelativeEncoder wristEncoder, leftEncoder, rightEncoder;

  public ClawSubsystem() {
    wristMotor = new SparkMax(ClawConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    leftMotor = new SparkMax(ClawConstants.LEFT_CLAW_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ClawConstants.RIGHT_CLAW_MOTOR_ID, MotorType.kBrushless);
    
    neoConfig = new SparkMaxConfig();
    neoConfig.smartCurrentLimit(ClawConstants.MOTOR_MAX_AMPERAGE);
    
    wristEncoder = wristMotor.getEncoder();
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    REVLibError wristError = wristMotor.configure(neoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    REVLibError leftError = leftMotor.configure(neoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    REVLibError rightError = rightMotor.configure(neoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    if (leftError == REVLibError.kOk && rightError == REVLibError.kOk && wristError == REVLibError.kOk) {
      wristEncoder.setPosition(0.0);
      leftEncoder.setPosition(0.0);
      rightEncoder.setPosition(0.0);
    } else {
      System.err.println("Claw Motor Configuration Failed!");
    }
  }

  public void tiltWristStop() {
    wristMotor.set(0.0);
  }

  public void tiltWristUp() {
    wristMotor.set(ClawConstants.WRIST_UP_SPEED);
  }

  public void tiltWristDown() {
    wristMotor.set(ClawConstants.WRIST_DOWN_SPEED);
  }

  public void stopRollers() {
    leftMotor.set(0.0);
    rightMotor.set(0.0);
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
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }

  @Override
  public void periodic() {
    
  }
}