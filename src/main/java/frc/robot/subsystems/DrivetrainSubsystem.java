// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.connection.ControllerConstants;
import frc.robot.constants.hardware.ChassisConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class DrivetrainSubsystem extends TunerSwerveDrivetrain implements Subsystem {

    public final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
    public final double maxAngularRate = Units.RotationsPerSecond.of(1.0).in(Units.RadiansPerSecond);

    public final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * ControllerConstants.DEADZONE)
                .withRotationalDeadband(maxAngularRate * ControllerConstants.DEADZONE)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public DrivetrainSubsystem(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        try {
            configureAutoBuilder();
        } catch (IOException | ParseException e) {
        }
    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? redAlliancePerspectiveRotation
                                : blueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void configureAutoBuilder() throws IOException, ParseException {
        ModuleConfig moduleConfig = new ModuleConfig(
                ChassisConstants.WHEEL_RADIUS,
                ChassisConstants.MAX_DRIVE_VELOCITY,
                ChassisConstants.WHEEL_COF,
                ChassisConstants.DRIVE_MOTOR,
                ChassisConstants.DRIVE_CURRENT_LIMIT,
                ChassisConstants.NUM_MOTORS
        );
        RobotConfig robotConfig = new RobotConfig(
                ChassisConstants.ROBOT_TOTAL_MASS,
                ChassisConstants.ROBOT_MOMENT_OF_INERTIA,
                moduleConfig,
                ChassisConstants.ROBOT_WIDTH
        );

        AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                        pathApplyRobotSpeeds.withSpeeds(speeds)
                                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                        new PIDConstants(3.3, 0, 0), //3.0, 0.0, 0.0
                        new PIDConstants(3, 0, 0)),
                robotConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
}
