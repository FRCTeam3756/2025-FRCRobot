package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    private static final HashMap<String, Command> autoModes = new HashMap<>();

    public static void init() {
        autoModes.put("Anywhere - Drive Forwards", driveStraightAuto());
        autoModes.put("Anywhere - Push Teammate on Left", leftDoubleAlgaeAuto());
        autoModes.put("Anywhere - Push Teammate on Right", rightDoubleAlgaeAuto());
        autoModes.put("Middle Align - Score Coral", middleScoreCoralAuto());
        autoModes.put("Left Align - Score Coral", leftScoreCoralAuto());
        autoModes.put("Left Align - Score 2 Algae", leftDoubleAlgaeAuto());
        autoModes.put("Right Align - Score Coral", rightScoreCoralAuto());
        autoModes.put("Right Align - Score 2 Algae", rightDoubleAlgaeAuto());

        SmartDashboard.putStringArray("Auto List", autoModes.keySet().toArray(new String[0]));
    }
    
    public static Command getSelectedAuto() {
        String selectedAuto = SmartDashboard.getString("Selected Auto", "Simple Path");

        return autoModes.getOrDefault(selectedAuto, driveStraightAuto());
    }

    public static Command driveStraightAuto() {
        return new PathPlannerAuto("DriveStraightAuto");
    }

    public static Command leftScoreCoralAuto() {
        return new PathPlannerAuto("LeftScoreCoralAuto");
    }

    public static Command middleScoreCoralAuto() {
        return new PathPlannerAuto("MiddleScoreCoralAuto");
    }

    public static Command rightScoreCoralAuto() {
        return new PathPlannerAuto("RightScoreCoralAuto");
    }

    public static Command leftDoubleAlgaeAuto() {
        return new PathPlannerAuto("LeftDoubleAlgaeAuto");
    }

    public static Command rightDoubleAlgaeAuto() {
        return new PathPlannerAuto("RightDoubleAlgaeAuto");
    }

    public static Command leftPushTeammateAuto() {
        return new PathPlannerAuto("LeftPushTeammateAuto");
    }

    public static Command rightPushTeammateAuto() {
        return new PathPlannerAuto("RightPushTeammateAuto");
    }
}
