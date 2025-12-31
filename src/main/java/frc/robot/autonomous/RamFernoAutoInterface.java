package frc.robot.autonomous;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface RamFernoAutoInterface {

    default Command followPath(String path) {
        try {
            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(path);
            return AutoBuilder.followPath(pathPlannerPath);
        } catch (ParseException | IOException e) {
            DriverStation.reportError("Failed to load path: " + path, e.getStackTrace());
            return Commands.none();
        }
    }
}
