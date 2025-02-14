package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.TunerConstants;
import frc.robot.swerve.*;

public class RobotContainer {
  private final Drive drive = new Drive(
    new GyroIOPigeon2(),
    new ModuleIOTalonFX(TunerConstants.FrontLeft),
    new ModuleIOTalonFX(TunerConstants.FrontRight),
    new ModuleIOTalonFX(TunerConstants.BackLeft),
    new ModuleIOTalonFX(TunerConstants.BackRight));
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
