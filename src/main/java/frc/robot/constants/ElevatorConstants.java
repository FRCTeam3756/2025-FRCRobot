package frc.robot.constants;

import java.util.Map;

public class ElevatorConstants {
    public static final double DEFAULT_ELEVATOR_SPEED = Integer.MAX_VALUE;

    public static final Map<Integer, Double> CORAL_SHOOT_HEIGHTS = Map.ofEntries(
        Map.entry(1, 0.0),      //TODO: Change numbers
        Map.entry(2, 1.0),
        Map.entry(3, 2.0),
        Map.entry(4, 3.0)
    );
}
