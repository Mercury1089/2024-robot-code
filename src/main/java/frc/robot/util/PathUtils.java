package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class PathUtils {
    public static Trajectory TrajectoryFromPath(PathPlannerTrajectory ppTrajectory) {
        List<Trajectory.State> states = new ArrayList<Trajectory.State>();
        for(PathPlannerTrajectory.State pState : ppTrajectory.getStates()) {
            states.add(new Trajectory.State(
                pState.timeSeconds,
                pState.velocityMps,
                pState.accelerationMpsSq,
                new Pose2d(pState.positionMeters, pState.heading),
                pState.curvatureRadPerMeter                
            ));
        }
        return new Trajectory(states);
    }
}
