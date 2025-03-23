package frc.robot.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoCommand extends Command {
    CommandSwerveDrivetrain drivetrain;
    Timer timer = new Timer();
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AutoCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.start();
        System.out.println("Command Starting");
    }

    @Override
    public void execute() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-0.4 * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))
                .withVelocityY(0.0 * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))
                .withRotationalRate(0.0 * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))));
    }

    public boolean isFinished() {
        if (timer.hasElapsed(2)) {
            System.out.println("Command Ending");
            return true;
        }
        return false;
    }
}
