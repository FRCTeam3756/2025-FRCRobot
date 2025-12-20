// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.connection.PortConstants;
import frc.robot.constants.subsystems.ClawConstants;
import frc.robot.constants.subsystems.ElevatorConstants;

public class ClawSubsystem extends SubsystemBase {
  private final SparkMax wristMotor;
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder wristEncoder;
  private final SparkMaxConfig wristConfig;

  public ClawSubsystem() {
    wristMotor = new SparkMax(PortConstants.WRIST_MOTOR_ID, ClawConstants.MOTOR_TYPE);
    leftMotor = new SparkMax(PortConstants.LEFT_CLAW_MOTOR_ID, ClawConstants.MOTOR_TYPE);
    rightMotor = new SparkMax(PortConstants.RIGHT_CLAW_MOTOR_ID, ClawConstants.MOTOR_TYPE);
    wristEncoder = wristMotor.getEncoder();
    wristConfig = new SparkMaxConfig();

    wristEncoder.setPosition(0);

    wristConfig
        .closedLoopRampRate(ClawConstants.MOTOR_RAMP_RATE)
        .idleMode(ClawConstants.IDLE_MODE)
        .smartCurrentLimit(ClawConstants.MOTOR_MAX_AMPERAGE).closedLoop
        .feedbackSensor(ClawConstants.FEEDBACK_SENSOR)
        .p(ClawConstants.P)
        .i(ClawConstants.I)
        .d(ClawConstants.D)
        .velocityFF(ClawConstants.FF)
        .outputRange(ClawConstants.MINIMUM_OUTPUT, ClawConstants.MAXIMUM_OUTPUT);

    wristMotor.configure(wristConfig, ElevatorConstants.RESET_MODE, ElevatorConstants.PERSIST_MODE);
  }

  @Override
  public void periodic() {}

  public void tiltWristUp() {
    // if (wristEncoder.getPosition() < ClawConstants.WRIST_MAX_HEIGHT) {
    wristMotor.set(ClawConstants.WRIST_UP_SPEED);
    // } else {
    // tiltWristStop();
    // }
  }

  public void tiltWristDown() {
    // if (wristEncoder.getPosition() > ClawConstants.WRIST_MIN_HEIGHT) {
    wristMotor.set(ClawConstants.WRIST_DOWN_SPEED);
    // } else {
    // tiltWristStop();
    // }
  }

  public void tiltWristStop() {
    wristMotor.set(0.0);
  }

  public void autoTiltWrist(double speed) {
    // if ((speed > 0) && (wristEncoder.getPosition() <
    // ClawConstants.WRIST_MAX_HEIGHT)) {
    wristMotor.set(speed);
    // } else if ((speed < 0) && (wristEncoder.getPosition() >
    // ClawConstants.WRIST_MIN_HEIGHT)) {
    // wristMotor.set(speed);
    // } else {
    // tiltWristStop();
    // }
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

  public void setRollerMotorSpeeds(double speed) {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }
}