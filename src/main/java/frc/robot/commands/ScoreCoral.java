// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreCoral extends Command {
    CoralSubsystem arm;
    int shootLevel;

    public ScoreCoral(CoralSubsystem subsystem, int shootLevel) {
        arm = subsystem;
        this.shootLevel = shootLevel;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        switch (this.shootLevel) {
            case 1:
                arm.shootL1();
                break;
            case 2:
                arm.shootL1();
                break;
            case 3:
                arm.shootL1();
                break;
            case 4:
                arm.shootL1();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
