// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootBarge extends Command {
  AlgaeSubsystem shooter;

  public ShootBarge(AlgaeSubsystem subsystem) {
    shooter = subsystem;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.shootBarge();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}