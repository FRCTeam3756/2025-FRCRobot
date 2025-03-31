// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends Command {
    private final ClawSubsystem claw;
    private final boolean intake;
    private double startTime;
    private final double duration, power;

    public ClawCommand(ClawSubsystem claw, double duration, double power, boolean intake) {
        this.intake = intake;
        this.duration = duration;
        this.claw = claw;
        this.power = power;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (intake) {
            claw.autoIntakeRollers(power);
        } else {
            claw.autoOuttakeRollers(power);
        }
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopRollers();
    }
}
