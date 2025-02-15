// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCoral extends Command {
  CoralSubsystem intake;

  public IntakeCoral(CoralSubsystem subystem) {
    intake = subystem;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.intakeGamePiece();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}