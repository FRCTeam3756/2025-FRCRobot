// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUp extends Command {
  ElevatorSubsystem subsystem;

  public ElevatorUp(ElevatorSubsystem elevator) {
    subsystem = elevator;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    subsystem.elevatorUp();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.elevatorStop();
  }
}