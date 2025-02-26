// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.swerve.Drive;

public class Controller {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final CommandXboxController controller = new CommandXboxController(Controller.DRIVER_CONTROLLER_PORT);

    public static final Trigger elevatorUpButton = controller.povRight();
    public static final Trigger elevatorDownButton = controller.povLeft();
    public static final Trigger climbButton = controller.povUp();
    public static final Trigger intakeButton = controller.rightBumper();
    public static final Trigger shootProcessorButton = controller.leftBumper();
    public static final Trigger shootCoralButton = controller.rightTrigger();
    public static final Trigger shootBargeButton = controller.leftTrigger();
    public static final Trigger autoIntakeButton = controller.a();
    public static final Trigger autoSpeakerButton = controller.b();
    public static final Trigger autoAmpButton = controller.x();
    public static final Trigger resetGyroButton = controller.y();
    public static final Trigger driveTurboButton = controller.leftStick();

    public static final double DEADZONE = 0.05;

    private static double applyDeadband(double value) {
        return MathUtil.applyDeadband(value, Controller.DEADZONE);
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = Math.hypot(x, y);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        linearMagnitude = applyDeadband(linearMagnitude);
        linearMagnitude *= linearMagnitude;

        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    private static double calculateRotation(double rot) {
        rot = applyDeadband(rot);
        return Math.copySign(rot * rot, rot);
    }

    private static boolean isAllianceFlipped() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rot) {
        return Commands.run(
                () -> {
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(x.getAsDouble(), y.getAsDouble());

                    double omega = calculateRotation(rot.getAsDouble());

                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    
                    boolean isFlipped = isAllianceFlipped();

                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                },
                drive);
    }
}
