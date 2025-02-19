package frc.robot.ai;

import java.util.HashMap;

import edu.wpi.first.networktables.*;

import frc.robot.constants.AIConstants;

public class JONCommunication {
    NetworkTableInstance networkTableInstance;
    NetworkTable networkTable;
    double x, y, rotation;
    boolean turbo, intake, outtake;
    long elevator, climb;

    /** Gets the desired position from the JON */
    public HashMap<String, Object> getAIRequest() {
        networkTableInstance = NetworkTableInstance.getDefault();
        networkTable = networkTableInstance.getTable(AIConstants.NETWORK_TABLE_NAME);

        x = networkTable.getDoubleTopic(AIConstants.NETWORK_TABLE_NAMING.X.getValue()).subscribe(0.0).get();
        y = networkTable.getDoubleTopic(AIConstants.NETWORK_TABLE_NAMING.Y.getValue()).subscribe(0.0).get();
        rotation = networkTable.getDoubleTopic(AIConstants.NETWORK_TABLE_NAMING.ROTATION.getValue()).subscribe(0.0).get();
        turbo = networkTable.getBooleanTopic(AIConstants.NETWORK_TABLE_NAMING.TURBO.getValue()).subscribe(false).get();
        intake = networkTable.getBooleanTopic(AIConstants.NETWORK_TABLE_NAMING.INTAKE.getValue()).subscribe(false).get();
        outtake = networkTable.getBooleanTopic(AIConstants.NETWORK_TABLE_NAMING.OUTTAKE.getValue()).subscribe(false).get();
        elevator = networkTable.getIntegerTopic(AIConstants.NETWORK_TABLE_NAMING.ELEVATOR.getValue()).subscribe(0).get();
        climb = networkTable.getIntegerTopic(AIConstants.NETWORK_TABLE_NAMING.CLIMB.getValue()).subscribe(0).get();
        
        HashMap<String, Object> hashMap = new HashMap<String, Object>();
        hashMap.put(AIConstants.NETWORK_TABLE_NAMING.X.getValue(), x);
        hashMap.put(AIConstants.NETWORK_TABLE_NAMING.Y.getValue(), y);
        hashMap.put(AIConstants.NETWORK_TABLE_NAMING.ROTATION.getValue(), rotation);
        hashMap.put(AIConstants.NETWORK_TABLE_NAMING.TURBO.getValue(), turbo);
        hashMap.put(AIConstants.NETWORK_TABLE_NAMING.INTAKE.getValue(), intake);
        hashMap.put(AIConstants.NETWORK_TABLE_NAMING.OUTTAKE.getValue(), outtake);
        hashMap.put(AIConstants.NETWORK_TABLE_NAMING.ELEVATOR.getValue(), elevator);
        hashMap.put(AIConstants.NETWORK_TABLE_NAMING.CLIMB.getValue(), climb);

        return hashMap;
    }
}