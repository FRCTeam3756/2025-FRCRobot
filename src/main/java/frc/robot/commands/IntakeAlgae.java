// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAlgae extends Command {
  AlgaeSubsystem subsystem;

  public IntakeAlgae(AlgaeSubsystem subystem) {
    this.subsystem = subystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    subsystem.intakeGamePiece();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }
}