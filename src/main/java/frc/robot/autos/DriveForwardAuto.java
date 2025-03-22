package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import frc.robot.Controller;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class DriveForwardAuto {
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond) * 0.1).withRotationalDeadband(Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond) * Controller.DEADZONE)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public void getAuto(double time, CommandSwerveDrivetrain drivetrain) {
        if (time < 0.0) {
            System.out.println("Waiting");
        } else if (time < 2.0) {
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(-0.5 * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))
                    .withVelocityY(0)
                    .withRotationalRate(0)
                );   
        }
    }
}