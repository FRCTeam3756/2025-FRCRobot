// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "Autonomous Program 1";
  private static final String auto2 = "Autonomous Program 2";
  private String selectedAuto;
  protected double startTime;

  public Robot() {
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    SmartDashboard.putData("Auto Choices", autoChooser);
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {} // TODO: STOP EVERYTHING!!!

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    selectedAuto = autoChooser.getSelected();
    System.out.println("Running autonomous program: " + selectedAuto);

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    switch (selectedAuto) {
      case auto1:
        robotContainer.getAuto1(Timer.getFPGATimestamp() - startTime).schedule();
        break;
      case auto2:
        break;
    }
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.setDriverControl();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }
}