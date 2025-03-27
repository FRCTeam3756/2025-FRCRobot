package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final Timer timer = new Timer();
    private final double duration, power;
    private boolean reversing = false;

    public ElevatorCommand(ElevatorSubsystem elevator, double power, double duration) {
        this.power = power;
        this.duration = duration;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (!reversing) {
            if (timer.get() < duration) {
                elevator.autoElevator(power);
            } else {
                reversing = true;
                timer.reset();
            }
        } else {
            if (timer.get() < duration) {
                elevator.autoElevator(-power);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return reversing && timer.get() >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.elevatorStop();
    }
}
