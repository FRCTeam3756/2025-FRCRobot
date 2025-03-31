// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double duration, power;
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
        if ((Timer.getFPGATimestamp() - startTime) < duration) {
            elevator.autoElevator(power);
        } else if ((Timer.getFPGATimestamp() - startTime) < duration * 2) {
            elevator.autoElevator(-power);
        }
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - startTime) >= duration * 2);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.elevatorStop();
    }
}
