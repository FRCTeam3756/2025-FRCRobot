// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkMaxConfig neoConfig;

  public ClawSubsystem() {
    leftMotor = new SparkMax(ClawConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ClawConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    
    neoConfig = new SparkMaxConfig();
    neoConfig.smartCurrentLimit(ClawConstants.NEO_MAX_AMPERAGE);

    REVLibError rightError = rightMotor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    REVLibError leftError = leftMotor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    if (leftError != REVLibError.kOk & rightError != REVLibError.kOk) {
      System.out.println("Issue with Claw Motors!");
    }
  }

  public void intakeGamePiece() {
    leftMotor.set(ClawConstants.INTAKE_SPEED);
    rightMotor.set(ClawConstants.INTAKE_SPEED);
  }

  public void outtakeGamePiece() {
    leftMotor.set(ClawConstants.OUTTAKE_SPEED);
    rightMotor.set(ClawConstants.OUTTAKE_SPEED);
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