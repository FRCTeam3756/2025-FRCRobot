// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.LocalADStarAK;

public class Robot extends TimedRobot {

    private final RobotContainer robotContainer;
    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    private Command autonomousCommand;

    public Robot() {
        robotContainer = new RobotContainer();

        autoChooser = robotContainer.buildAutoChooser(); // Create auto selector
        SmartDashboard.putData("Auto List", autoChooser); // Display it to SmartDashboard

        Pathfinding.setPathfinder(new LocalADStarAK()); // Set the Pathfinder to be compatible with AdvantageKit
        PathfindingCommand.warmupCommand().schedule(); // Warmup the Pathfinder

        DataLogManager.start();
        Logger.recordMetadata("Team", "3756");
        Logger.recordMetadata("Robot", "RamFerno Swerve");

        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter(
                    LogFileUtil.findReplayLog()));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        SignalLogger.start();
    }

    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run();
        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = autoChooser.getSelected();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }
}
