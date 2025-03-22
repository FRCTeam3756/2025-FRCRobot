package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.*;

public class PushRightTeammateAuto {
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public void runAuto(double time, CommandSwerveDrivetrain drivetrain) {
        if (time < 0.0) {
            System.out.println("Waiting");
        } else if (time < 2.0) {
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(-0.5 * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))
                    .withVelocityY(0)
                    .withRotationalRate(0)
                );            
        } else if (time < 4.0) {
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0)
                .withVelocityY(-0.5 * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))
                .withRotationalRate(0)
            );
        } else if (time < 8.0) {
            drivetrain.applyRequest(() ->
                drive.withVelocityX(1.0 * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))
                .withVelocityY(0)
                .withRotationalRate(0)
            );
        } else {
            drivetrain.applyRequest(() -> brake);
        }
    }
}