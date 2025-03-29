package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClawSubsystem;

public class WristCommand extends Command {
    private final ClawSubsystem claw;
    private final double duration, power;
    private boolean reversing = false;
    private double startTime;

    public WristCommand(ClawSubsystem claw, double power, double duration) {
        this.power = power;
        this.duration = duration;
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (!reversing) {
            if ((Timer.getFPGATimestamp() - startTime) < duration) {
                claw.autoTiltWrist(power);
            } else {
                reversing = true;
            }
        } else {
            if (((Timer.getFPGATimestamp() - startTime) - duration) < duration) {
                claw.autoTiltWrist(-power);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return reversing && ((Timer.getFPGATimestamp() - startTime) >= duration);
    }

    @Override
    public void end(boolean interrupted) {
        claw.tiltWristStop();
    }
}
