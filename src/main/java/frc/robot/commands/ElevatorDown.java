// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDown extends Command {
  ElevatorSubsystem subsystem;
  int elevatorHeight;

  public ElevatorDown(ElevatorSubsystem subsystem, int elevatorHeight) {
    this.subsystem = subsystem;
    this.elevatorHeight = elevatorHeight;
    addRequirements(this.subsystem);
  }

  @Override
  public void execute() {
    subsystem.elevatorDown(elevatorHeight);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.elevatorStop(elevatorHeight);
  }
}