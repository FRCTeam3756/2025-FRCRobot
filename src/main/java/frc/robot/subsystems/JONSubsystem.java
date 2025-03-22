// // Copyright (c) FRC Team 3756 RamFerno.
// // Open Source Software; you can modify and/or share it under the terms of
// // the license viewable in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.util.HashMap;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;

// import frc.robot.constants.JONConstants;

// public class JONSubsystem extends SubsystemBase {
//     NetworkTable networkTable;

//     public JONSubsystem() {}

//     @Override
//     public void periodic() {
//         sendMatchPhaseToJON();
//         getJONData();
//     }

//     public void sendMatchPhaseToJON() {
//         String matchPhase = DriverStation.isAutonomousEnabled() ? "Auto" :
//                             DriverStation.isTeleopEnabled() ? "Tele-Op" : "Pre-Match";

//         sendJONData("MATCH_PHASE", matchPhase);
//     }
    
//     public HashMap<String, Object> getJONData() {
//         networkTable = NetworkTableInstance.getDefault().getTable(JONConstants.NETWORK_TABLE_NAME);
//         double xPercentage = networkTable.getEntry(JONConstants.NETWORK_TABLE_NAMING.X.getValue()).getDouble(0.0);
//         double yPercentage = networkTable.getEntry(JONConstants.NETWORK_TABLE_NAMING.Y.getValue()).getDouble(0.0);
//         double rotationPercentage = networkTable.getEntry(JONConstants.NETWORK_TABLE_NAMING.ROTATION.getValue()).getDouble(0.0);
//         boolean turboState = networkTable.getEntry(JONConstants.NETWORK_TABLE_NAMING.TURBO.getValue()).getBoolean(false);
//         boolean intakeState = networkTable.getEntry(JONConstants.NETWORK_TABLE_NAMING.INTAKE.getValue()).getBoolean(false);
//         boolean outtakeState = networkTable.getEntry(JONConstants.NETWORK_TABLE_NAMING.OUTTAKE.getValue()).getBoolean(false);
//         boolean climbState = networkTable.getEntry(JONConstants.NETWORK_TABLE_NAMING.CLIMB.getValue()).getBoolean(false);
//         int elevatorStage = (int) networkTable.getEntry(JONConstants.NETWORK_TABLE_NAMING.ELEVATOR.getValue()).getInteger(0);
        
//         HashMap<String, Object> hashmap = new HashMap<>();
//         hashmap.put(JONConstants.NETWORK_TABLE_NAMING.X.getValue(), xPercentage);
//         hashmap.put(JONConstants.NETWORK_TABLE_NAMING.Y.getValue(), yPercentage);
//         hashmap.put(JONConstants.NETWORK_TABLE_NAMING.ROTATION.getValue(), rotationPercentage);
//         hashmap.put(JONConstants.NETWORK_TABLE_NAMING.TURBO.getValue(), turboState);
//         hashmap.put(JONConstants.NETWORK_TABLE_NAMING.INTAKE.getValue(), intakeState);
//         hashmap.put(JONConstants.NETWORK_TABLE_NAMING.OUTTAKE.getValue(), outtakeState);
//         hashmap.put(JONConstants.NETWORK_TABLE_NAMING.ELEVATOR.getValue(), elevatorStage);
//         hashmap.put(JONConstants.NETWORK_TABLE_NAMING.CLIMB.getValue(), climbState);

//         return hashmap;
//     }
    
//     public void sendJONData(String name, Object data) {
//         networkTable = NetworkTableInstance.getDefault().getTable(JONConstants.NETWORK_TABLE_NAME);
//         networkTable.getEntry(name).setValue(data);
//     }
// }
