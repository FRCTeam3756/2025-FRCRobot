// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  private static enum auto {
    driveForward,
    pushLeftTeammate,
    pushRightTeammate,
    pickupAlgaeLeft,
    pickupAlgaeMiddle,
    pickupAlgaeRight
  };

  private auto selectedAuto = auto.pushRightTeammate;
  private double startTime;

  public Robot() {
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
    startTime = Timer.getFPGATimestamp();
    System.out.println("Running autonomous program: " + selectedAuto);

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    switch (selectedAuto) {
      case driveForward:
        autonomousCommand = robotContainer.getDriveForwardAuto(Timer.getFPGATimestamp() - startTime);
        if (autonomousCommand != null) {
          autonomousCommand.schedule();
        }
        break;
      case pushLeftTeammate:
        autonomousCommand = robotContainer.getPushLeftAuto(Timer.getFPGATimestamp() - startTime);
        if (autonomousCommand != null) {
          autonomousCommand.schedule();
        }
        break;
      case pushRightTeammate:
        autonomousCommand = robotContainer.getPushRightAuto(Timer.getFPGATimestamp() - startTime);
        if (autonomousCommand != null) {
          autonomousCommand.schedule();
        }
        break;
      case pickupAlgaeLeft:
        autonomousCommand = robotContainer.getLeftPickupAlgaeAuto(Timer.getFPGATimestamp() - startTime);
        if (autonomousCommand != null) {
          autonomousCommand.schedule();
        }
        break;
      case pickupAlgaeMiddle:
        autonomousCommand = robotContainer.getMiddlePickupAlgaeAuto(Timer.getFPGATimestamp() - startTime);
        if (autonomousCommand != null) {
          autonomousCommand.schedule();
        }
        break;
      case pickupAlgaeRight:
        autonomousCommand = robotContainer.getRightPickupAlgaeAuto(Timer.getFPGATimestamp() - startTime);
        if (autonomousCommand != null) {
          autonomousCommand.schedule();
        }
        break;
      default:
        autonomousCommand.schedule();
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