package frc.robot.ai;

import java.util.HashMap;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.AIConstants;

public class JONCommunication {
    NetworkTableInstance networkTableInstance;
    NetworkTable networkTable;
    DoubleSubscriber xSub, ySub, rotSub;
    double x, y, rot;

    /** Gets the desired position from the JON */
    public HashMap<String, Double> updateValues() {
        networkTableInstance = NetworkTableInstance.getDefault();
        networkTable = networkTableInstance.getTable(AIConstants.NETWORK_TABLE_NAME);
        xSub = networkTable.getDoubleTopic(AIConstants.X_NAME).subscribe(0.0);
        ySub = networkTable.getDoubleTopic(AIConstants.Y_NAME).subscribe(0.0);
        rotSub = networkTable.getDoubleTopic(AIConstants.ROT_NAME).subscribe(0.0);
        
        x = xSub.get();
        y = ySub.get();
        rot = rotSub.get();
        
        HashMap<String, Double> hashMap = new HashMap<String, Double>();
        hashMap.put(AIConstants.X_NAME, x);
        hashMap.put(AIConstants.Y_NAME, y);
        hashMap.put(AIConstants.ROT_NAME, rot);

        return hashMap;
    }
}