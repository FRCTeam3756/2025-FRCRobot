// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

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

  private auto selectedAuto = auto.driveForward;
  private double startTime;

  public Robot() {
    robotContainer = new RobotContainer();
    robotContainer.setDriverControl();
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

    robotContainer.getPushRightAuto(Timer.getFPGATimestamp() - startTime);
  }

  @Override
  public void autonomousPeriodic() {
    switch (selectedAuto) {
      case driveForward:
        // robotContainer.getDriveForwardAuto(Timer.getFPGATimestamp() - startTime);
        break;
      case pushLeftTeammate:
        robotContainer.getPushLeftAuto(Timer.getFPGATimestamp() - startTime);
        break;
      case pushRightTeammate:
        robotContainer.getPushRightAuto(Timer.getFPGATimestamp() - startTime);
        break;
      case pickupAlgaeLeft:
        robotContainer.getLeftPickupAlgaeAuto(Timer.getFPGATimestamp() - startTime);
        break;
      case pickupAlgaeMiddle:
        robotContainer.getMiddlePickupAlgaeAuto(Timer.getFPGATimestamp() - startTime);
        break;
      case pickupAlgaeRight:
        robotContainer.getRightPickupAlgaeAuto(Timer.getFPGATimestamp() - startTime);
        break;
      default:
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