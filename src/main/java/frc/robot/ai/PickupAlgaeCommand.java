package frc.robot.ai;

import frc.robot.subsystems.JONSubsystem;
import frc.robot.swerve.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class PickupAlgaeCommand extends Command {
  private final Drive drive;
  private final JONSubsystem jonSubsystem;

  public PickupAlgaeCommand(Drive drive, JONSubsystem jonSubsystem) {
    this.drive = drive;
    this.jonSubsystem = jonSubsystem;
    addRequirements(drive, jonSubsystem);
  }

  @Override
  public void initialize() {
    drive.setDefaultCommand(
      new RunCommand(
                    () -> drive.drive(
                        drive,
                        getX(),
                        getY(),
                        getRotation()
                    ),
                    drive
      )
    );
    jonSubsystem.autoPickupAlgae();
  }

  @Override
  public void execute(
  ) {}

  @Override
  public void end(boolean interrupted) {
    jonSubsystem.stopRollers();
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double getX() {
    return 0.0;
  }
  public double getY() {
    return 0.0;
  }
  public double getRotation() {
    return 0.0;
  }
}
