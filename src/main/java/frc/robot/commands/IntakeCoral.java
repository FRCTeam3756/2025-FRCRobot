// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCoral extends Command {
  ClawSubsystem subsystem;

  public IntakeCoral(ClawSubsystem subystem) {
    this.subsystem = subystem;
    addRequirements(this.subsystem);
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