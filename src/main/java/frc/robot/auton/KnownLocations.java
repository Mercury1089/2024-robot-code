// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/** Absolute (X, Y) of certain field locations
 * (thanks design :)
 */
public class KnownLocations {

    // do nothing auton
    public static final Pose2d
        DO_NOTHING = PathPointInch(0, 0, 0);

    // looking from its alliance side
    //TODO: change x values of these starting positions based on testing
    public final Pose2d 
        START_LEFT_NOTE,
        START_MID_NOTE,
        START_RIGHT_NOTE;

    // notes -> NOTE_WING_1 lines up w/NOTE_CENTER_1 from the top 
    public final Pose2d
        WING_NOTE_1,
        WING_NOTE_2,
        WING_NOTE_3,
        CENTER_NOTE_1,
        CENTER_NOTE_2,
        CENTER_NOTE_3,
        CENTER_NOTE_4,
        CENTER_NOTE_5;
    
    //field -> SUBWOOFER_1 points w/NOTE_WING_1
    // public final Pose2d
    //     AMP,
    //     SUBWOOFER_1,
    //     SUBWOOFER_2,
    //     SUBWOOFER_3;
        
    public Optional<Alliance> allianceColor = DriverStation.getAlliance();

    public KnownLocations() {

        allianceColor = DriverStation.getAlliance();

        if (allianceColor.isPresent() && allianceColor.get() == Alliance.Blue) {

            WING_NOTE_1 = PathPointInch(114.261, 276, 0);
            WING_NOTE_2 = PathPointInch(114.261, 219, 0);
            WING_NOTE_3 = PathPointInch(114.261, 162, 0);

            CENTER_NOTE_1 = PathPointInch(325.862, 30, 180);
            CENTER_NOTE_2 = PathPointInch(325.862, 96, 180);
            CENTER_NOTE_3 = PathPointInch(325.862, 162, 180);
            CENTER_NOTE_4 = PathPointInch(325.862, 228, 180);
            CENTER_NOTE_5 = PathPointInch(325.862, 294, 180);

            START_LEFT_NOTE = PathPointInch(50, WING_NOTE_1.getY(), 180);
            START_MID_NOTE = PathPointInch(50, WING_NOTE_2.getY(), 180);
            START_RIGHT_NOTE = PathPointInch(50, WING_NOTE_3.getY(), 180);

        } else {

            WING_NOTE_1 = PathPointInch(537.464, 276, 180);
            WING_NOTE_2 = PathPointInch(537.464, 219, 180);
            WING_NOTE_3 = PathPointInch(537.464, 162, 180);

            CENTER_NOTE_1 = PathPointInch(325.862, 30, 0);
            CENTER_NOTE_2 = PathPointInch(325.862, 96, 0);
            CENTER_NOTE_3 = PathPointInch(325.862, 162, 0);
            CENTER_NOTE_4 = PathPointInch(325.862, 228, 0);
            CENTER_NOTE_5 = PathPointInch(325.862, 294, 0);

            START_LEFT_NOTE = PathPointInch(600, WING_NOTE_1.getY(), 0);
            START_MID_NOTE = PathPointInch(600, WING_NOTE_2.getY(), 0);
            START_RIGHT_NOTE = PathPointInch(600, WING_NOTE_3.getY(), 0);
        }
    }

    /** Convenience method to create PathPoint from inches */
    private static Pose2d PathPointInch(double xInches, double yInches, double headingDegrees) {
        return new Pose2d(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)), Rotation2d.fromDegrees(headingDegrees));
    }

    /** Convenience method to create waypoint excluding holonomic rotation */
    private static Pose2d PathWayPoint(double xInches, double yInches, double headingDegrees) {
        return new Pose2d(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)), Rotation2d.fromDegrees(headingDegrees));
    }

}
