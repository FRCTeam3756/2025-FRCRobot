package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class RunRollersCommand extends Command {

    private final ClawSubsystem clawSubsystem;
    private final boolean runIntake;
    private final boolean runOuttake;

    public RunRollersCommand(ClawSubsystem clawSubsystem, boolean runIntake, boolean runOuttake) {
        this.clawSubsystem = clawSubsystem;
        this.runIntake = runIntake;
        this.runOuttake = runOuttake;

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        if (runIntake) {
            clawSubsystem.intakeRollers();
        } else if (runOuttake) {
            clawSubsystem.outtakeRollers();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stopRollers();
    }
}
