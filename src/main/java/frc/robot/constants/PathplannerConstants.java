package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class PathplannerConstants {
    public static final double ROBOT_TOTAL_MASS = 60; //kg

    public static final Distance WHEEL_RADIUS = Distance.ofBaseUnits(0.051, Units.Meters); // m
    public static final LinearVelocity MAX_DRIVE_VELOCITY = LinearVelocity.ofBaseUnits(5.900, Units.MetersPerSecond);
    public static final double WHEEL_COF = 1.200;
    public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
    public static final Current DRIVE_CURRENT_LIMIT = Current.ofBaseUnits(40, Units.Amps);
    public static final int NUM_MOTORS = 1;

    private static final double DIAGONAL_WHEEL_CENTRE_DISTANCE = 0.81; // m
    public static final double ROBOT_WIDTH = 0.7112; // m

    private static final double ANGULAR_ACCELERATION_FEEDFORWARD = 4; // V/(rad/s^2)
    private static final double LINEAR_ACCELERATION_FEEDFORWARD = 4; // V/(m/s^2)
    public static final double ROBOT_MOMENT_OF_INERTIA = ROBOT_TOTAL_MASS * (DIAGONAL_WHEEL_CENTRE_DISTANCE / 2) * (ANGULAR_ACCELERATION_FEEDFORWARD / LINEAR_ACCELERATION_FEEDFORWARD); // KG*M^2
}