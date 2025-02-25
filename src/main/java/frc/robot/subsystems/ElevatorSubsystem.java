// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.ClimbConstants;

public class ElevatorSubsystem extends SubsystemBase{
  private SparkMax elevatorMotor;
  private SparkMaxConfig neoConfig;
  
  public ElevatorSubsystem() {
    elevatorMotor = new SparkMax(ClimbConstants.LEFT_CAN_ID, MotorType.kBrushless);
    
    neoConfig = new SparkMaxConfig();
    neoConfig.smartCurrentLimit(ClawConstants.NEO_MAX_AMPERAGE);

    REVLibError error = elevatorMotor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    if (error != REVLibError.kOk) {
      System.out.println("Issue with Elevator Motor!");
    }
  }

  public void elevatorUp(int elevatorHeight) {
    if (elevatorHeight >= 4) {
      return;
    }
    elevatorMotor.set(ClimbConstants.CLIMB_SPEED);
  }

  public void elevatorDown(int elevatorHeight) {
    if (elevatorHeight <= 1) {
      return;
    }
    elevatorMotor.set(-ClimbConstants.CLIMB_SPEED);
  }

  public void elevatorStop(int elevatorHeight) {
    elevatorMotor.set(0);
  }

  @Override
  public void periodic() {}
}
