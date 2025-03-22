package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import frc.robot.Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PushLeftTeammateAuto {
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond) * 0.1).withRotationalDeadband(Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond) * Controller.DEADZONE)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public void getAuto(double time, CommandSwerveDrivetrain drivetrain) {
        if (time < 0.0) {

        } else if (time < 2.0) {
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-0.5)
                .withVelocityY(0)
                .withRotationalRate(0));
        } else if (time < 4.0) {
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0)
                .withVelocityY(-0.5)
                .withRotationalRate(0));
        } else if (time < 8.0) {
            drivetrain.applyRequest(() ->
                drive.withVelocityX(1.0)
                .withVelocityY(0)
                .withRotationalRate(0));
        } else {
            drivetrain.applyRequest(() -> brake);
        }
    }
}