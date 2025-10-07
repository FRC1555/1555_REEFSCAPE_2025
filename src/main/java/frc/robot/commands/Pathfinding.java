package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.util.FileVersionException;

public class Pathfinding {
    /**
     * Returns a command that pathfinds to the start of 
     * a given path, and then follows said path.
     * 
     * @param pathName The name of the .path file (without extension)
     * @return A Command ready to schedule
     */
    public static Command goTo(String pathName) {
        try {
            // Load your saved PathPlanner path (must exist in src/main/deploy/pathplanner/)
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // Use that path’s internal constraints (or override with your own)
            PathConstraints constraints = path.getGlobalConstraints();

            // Chain: pathfind to the start, then follow path
            return AutoBuilder.pathfindThenFollowPath(path, constraints);

        } catch (IOException | ParseException | FileVersionException e) {
            DriverStation.reportError(
                "❌ Failed to load PathPlanner path: " + pathName + " (" + e.getMessage() + ")",
                e.getStackTrace()
            );
            // Return a do-nothing command if load fails
            return Commands.none();
        }
    }
}
