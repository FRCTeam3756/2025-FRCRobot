// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAlgae extends Command {
  AlgaeSubsystem intake;

  public IntakeAlgae(AlgaeSubsystem subystem) {
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