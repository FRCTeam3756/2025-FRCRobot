package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class RobotConstants {

    public static final double ROBOT_TOTAL_MASS = 60; //kg

    public static final Distance WHEEL_RADIUS = Distance.ofBaseUnits(0.051, Units.Meters); // m
    public static final LinearVelocity MAX_DRIVE_VELOCITY = LinearVelocity.ofBaseUnits(5.900, Units.MetersPerSecond);
    public static final double WHEEL_COF = 1.200;
    public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
    public static final Current DRIVE_CURRENT_LIMIT = Current.ofBaseUnits(40, Units.Amps);
    public static final int NUM_MOTORS = 1;

    public static final double DIAGONAL_WHEEL_CENTRE_DISTANCE = 0.81; // m
    public static final double ROBOT_WIDTH = 0.7112; // m

    public static final double ANGULAR_ACCELERATION_FEEDFORWARD = 4; // V/(rad/s^2)
    public static final double LINEAR_ACCELERATION_FEEDFORWARD = 4; // V/(m/s^2)
    public static final double ROBOT_MOMENT_OF_INERTIA = ROBOT_TOTAL_MASS * (DIAGONAL_WHEEL_CENTRE_DISTANCE / 2) * (ANGULAR_ACCELERATION_FEEDFORWARD / LINEAR_ACCELERATION_FEEDFORWARD); // KG*M^2

    public static final double MAX_AUTONOMOUS_VELOCITY = 3.0; // m/s
    public static final double MAX_AUTONOMOUS_ACCELERATION = 4.0; // m/s^2
    public static final double MAX_AUTONOMOUS_ANGULAR_VELOCITY = 3 * Math.PI; // rad/s
    public static final double MAX_AUTONOMOUS_ANGULAR_ACCELERATION = 4 * Math.PI; // rad/s^2

    public static final Map<String, Transform2d> CAMERA_TRANSFORMS = Map.ofEntries(
            Map.entry("Center",
                    new Transform2d(
                            new Translation2d(0.30, 0.00),
                            Rotation2d.fromDegrees(0)
                    )
            ),
            Map.entry("Left",
                    new Transform2d(
                            new Translation2d(0.25, 0.20),
                            Rotation2d.fromDegrees(45)
                    )
            ),
            Map.entry("Right",
                    new Transform2d(
                            new Translation2d(0.25, -0.20),
                            Rotation2d.fromDegrees(-45)
                    )
            )
    );
}
