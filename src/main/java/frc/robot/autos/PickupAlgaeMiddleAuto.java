package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class PickupAlgaeMiddleAuto {
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond) * 0.1).withRotationalDeadband(Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond) * Controller.DEADZONE)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();

    public Command getAuto(double time, CommandSwerveDrivetrain drivetrain) {
        if (time < 0.0) {
            return null;
        } else if (time < 2.0) {
            elevatorSubsystem.elevatorUp();
            return drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                .withVelocityY(0)
                .withRotationalRate(0));
        } else if (time < 5.0) {
            clawSubsystem.intakeGamePiece();
            return drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                .withVelocityY(-0.25)
                .withRotationalRate(0));
        }
    
        return null;
    }
}