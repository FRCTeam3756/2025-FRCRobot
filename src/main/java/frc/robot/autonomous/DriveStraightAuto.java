package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.logic.ControlManager;
import frc.robot.logic.GoalManager;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraightAuto extends SequentialCommandGroup implements RamFernoAutoInterface {

    public DriveStraightAuto(
            ControlManager controlManager,
            DrivetrainSubsystem drivetrainSubsystem,
            GoalManager goalManager
    ) {
        addCommands(
                followPath("LeftDoubleAlgaePath1")
        );
    }
}
