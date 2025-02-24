package frc.robot.ai;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.AIConstants;

public class JONCommunication {
    NetworkTable networkTable;
    double xPercentage, yPercentage, rotationPercentage;
    boolean turboState, intakeState, outtakeState;
    long elevatorStage, climbState;

    /** Gets the desired position from the JON */
    public HashMap<String, Object> getAIData() {
        networkTable = NetworkTableInstance.getDefault().getTable(AIConstants.NETWORK_TABLE_NAME);

        xPercentage = networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.X.getValue()).getDouble(0.0);
        yPercentage = networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.Y.getValue()).getDouble(0.0);
        rotationPercentage = networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.ROTATION.getValue()).getDouble(0.0);
        turboState = networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.TURBO.getValue()).getBoolean(false);
        intakeState = networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.INTAKE.getValue()).getBoolean(false);
        outtakeState = networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.OUTTAKE.getValue()).getBoolean(false);
        elevatorStage = networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.ELEVATOR.getValue()).getInteger(0);
        climbState = networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.CLIMB.getValue()).getInteger(0);
        
        HashMap<String, Object> hashmap = new HashMap<>();
        hashmap.put(AIConstants.NETWORK_TABLE_NAMING.X.getValue(), xPercentage);
        hashmap.put(AIConstants.NETWORK_TABLE_NAMING.Y.getValue(), yPercentage);
        hashmap.put(AIConstants.NETWORK_TABLE_NAMING.ROTATION.getValue(), rotationPercentage);
        hashmap.put(AIConstants.NETWORK_TABLE_NAMING.TURBO.getValue(), turboState);
        hashmap.put(AIConstants.NETWORK_TABLE_NAMING.INTAKE.getValue(), intakeState);
        hashmap.put(AIConstants.NETWORK_TABLE_NAMING.OUTTAKE.getValue(), outtakeState);
        hashmap.put(AIConstants.NETWORK_TABLE_NAMING.ELEVATOR.getValue(), elevatorStage);
        hashmap.put(AIConstants.NETWORK_TABLE_NAMING.CLIMB.getValue(), climbState);

        return hashmap;
    }

    public void sendJONData(HashMap<String, Object> hashmap) {
        networkTable = NetworkTableInstance.getDefault().getTable(AIConstants.NETWORK_TABLE_NAME);

        networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.X.getValue()).setDouble((double) hashmap.get(AIConstants.NETWORK_TABLE_NAMING.X.getValue()));
        networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.Y.getValue()).setDouble((double) hashmap.get(AIConstants.NETWORK_TABLE_NAMING.Y.getValue()));
        networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.ROTATION.getValue()).setDouble((double) hashmap.get(AIConstants.NETWORK_TABLE_NAMING.ROTATION.getValue()));
        networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.TURBO.getValue()).setBoolean((boolean) hashmap.get(AIConstants.NETWORK_TABLE_NAMING.TURBO.getValue()));
        networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.INTAKE.getValue()).setBoolean((boolean) hashmap.get(AIConstants.NETWORK_TABLE_NAMING.INTAKE.getValue()));
        networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.OUTTAKE.getValue()).setBoolean((boolean) hashmap.get(AIConstants.NETWORK_TABLE_NAMING.OUTTAKE.getValue()));
        networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.ELEVATOR.getValue()).setInteger((long) hashmap.get(AIConstants.NETWORK_TABLE_NAMING.ELEVATOR.getValue()));
        networkTable.getEntry(AIConstants.NETWORK_TABLE_NAMING.CLIMB.getValue()).setInteger((long) hashmap.get(AIConstants.NETWORK_TABLE_NAMING.CLIMB.getValue()));
    }
}