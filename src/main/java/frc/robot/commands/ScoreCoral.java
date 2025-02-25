// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreCoral extends Command {
    ClawSubsystem subsystem;
    int shootLevel;

    public ScoreCoral(ClawSubsystem subsystem, int shootLevel) {
        this.subsystem = subsystem;
        this.shootLevel = shootLevel;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        switch (this.shootLevel) {
            case 1:
                subsystem.shootL1();
                break;
            case 2:
                subsystem.shootL1();
                break;
            case 3:
                subsystem.shootL1();
                break;
            case 4:
                subsystem.shootL1();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
