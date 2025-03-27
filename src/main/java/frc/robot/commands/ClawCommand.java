package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends Command {
    private final ClawSubsystem claw;
    private final boolean intake;
    private final Timer timer = new Timer();
    private final double duration;

    public ClawCommand(ClawSubsystem claw, double duration, boolean intake) {
        this.intake = intake;
        this.duration = duration;
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (intake) {
            claw.intakeRollers();
        } else {
            claw.outtakeRollers();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopRollers();
    }
}
