package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double duration, power;
    private boolean reversing = false;
    private double startTime;

    public ElevatorCommand(ElevatorSubsystem elevator, double power, double duration) {
        this.power = power;
        this.duration = duration;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (!reversing) {
            if ((Timer.getFPGATimestamp() - startTime) < duration) {
                elevator.autoElevator(power);
            } else {
                reversing = true;
            }
        } else {
            if (((Timer.getFPGATimestamp() - startTime) - duration) < duration) {
                elevator.autoElevator(-power);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return reversing && ((Timer.getFPGATimestamp() - startTime) >= duration);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.elevatorStop();
    }
}
