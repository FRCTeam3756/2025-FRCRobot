// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final RobotContainer robotContainer;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private Command autonomousCommand;

  public Robot() {
    robotContainer = new RobotContainer();
    robotContainer.setDriverControl();
  }

  @Override
  public void robotInit() {
    UsbCamera camera = CameraServer.startAutomaticCapture();

    camera.setResolution(640, 480);
    camera.setFPS(30);

    CameraServer.startAutomaticCapture(camera);

    autoChooser.setDefaultOption("Anywhere - Drive Forwards", "DriveStraightAuto");
    autoChooser.addOption("Middle Align - Score Coral", "MiddleScoreCoralAuto");
    autoChooser.addOption("Left Align - Score Coral", "LeftScoreCoralAuto");
    autoChooser.addOption("Left Align - Score 2 Algae", "LeftDoubleAlgaeAuto");
    autoChooser.addOption("Left Align - Push Teammate", "LeftPushTeammateAuto");
    autoChooser.addOption("Right Align - Score Coral", "RightScoreCoralAuto");
    autoChooser.addOption("Right Align - Score 2 Algae", "RightDoubleAlgaeAuto");
    autoChooser.addOption("Right Align - Push Teammate", "RightPushTeammateAuto");

    SmartDashboard.putData("Auto List", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {
  } // TODO: STOP EVERYTHING!!!

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

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
    robotContainer.setDriverControl();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }
}