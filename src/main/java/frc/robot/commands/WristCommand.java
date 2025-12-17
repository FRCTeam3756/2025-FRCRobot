// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class WristCommand extends Command {
    private final ClawSubsystem claw;
    private final double duration, power, timeout;
    private double startTime;

    public WristCommand(ClawSubsystem claw, double power, double duration, double timeout) {
        this.power = power;
        this.duration = duration;
        this.claw = claw;
        this.timeout = timeout;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if ((Timer.getFPGATimestamp() - startTime) < duration) {
            claw.autoTiltWrist(power);
        } else {
            if (((Timer.getFPGATimestamp() - startTime) + timeout) < (duration * 2) + timeout) {
                claw.autoTiltWrist(-power);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - startTime) + timeout) >= ((duration * 2) + timeout);
    }

    @Override
    public void end(boolean interrupted) {}
}
