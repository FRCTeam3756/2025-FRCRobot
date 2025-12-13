// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final RobotContainer robotContainer;
  private SendableChooser<String> autoChooser = new SendableChooser<>();
  private Command autonomousCommand;

  public Robot() {
    robotContainer = new RobotContainer();
    autoChooser = robotContainer.buildAutoChooser();
    SmartDashboard.putData("Auto List", autoChooser);
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    String selectedAuto = autoChooser.getSelected();
    System.out.println(selectedAuto);

    if (selectedAuto != null) {
      autonomousCommand = new PathPlannerAuto(selectedAuto);
      if (autonomousCommand != null) {
        autonomousCommand.schedule();
      }
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}
}