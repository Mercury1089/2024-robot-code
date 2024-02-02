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

    // auton starts
    public final Pose2d 
        START_TOPMOST,
        START_TOP_SECOND,
        START_BOTTOM_SECOND,
        START_BOTTOMMOST,
        START_MIDDLE_CONE;
    
    // elements
    public final Pose2d
        ELEMENT1,
        ELEMENT2,
        ELEMENT3,
        ELEMENT4;
    
    //charging station
    public final Pose2d
        CHARGING_MIDDLE_CONE,
        CHARGING_CENTER,
        CHARGING_TOP_LEFT,
        CHARGING_TOP_RIGHT,
        CHARGING_BOTTOM_LEFT,
        CHARGING_BOTTOM_RIGHT;

    public final Pose2d
        WAYPOINT_TOP,
        WAYPOINT_BOTTOM,
        WAYPOINT_CHARGING;
        // WAYPOINT_CHARGING_BACK;
    
        
    public Optional<Alliance> allianceColor = DriverStation.getAlliance();

    public KnownLocations() {

        allianceColor = DriverStation.getAlliance();

        if ( allianceColor.isPresent() && allianceColor.get() == Alliance.Blue) {
            START_TOPMOST = PathPointInch(54.93+16.5, 199.65, 0);
            START_TOP_SECOND = PathPointInch(54.93+16.5, 173.52, 0);
            START_BOTTOM_SECOND = PathPointInch(54.93+16.5, 41.67, 0);
            START_BOTTOMMOST = PathPointInch(54.93+16.5, 16.15, 0);
            START_MIDDLE_CONE = PathPointInch(54.93+16.5, 129.9, 0);
            // START_MIDDLE_CONE = new PathPoint(
            //     new Translation2d(
            //         START_TOPMOST.position.getX(),
            //         ((START_TOPMOST.position.getY() + START_BOTTOMMOST.position.getY()) / 2.0) + 22.0
            //     ),
            //     Rotation2d.fromDegrees(0),
            //     Rotation2d.fromDegrees(180)
            // );

            ELEMENT1 = PathPointInch(279.31-27.5, 180.02, 0);
            ELEMENT2 = PathPointInch(279.31-27.5, 132.02, 0);
            ELEMENT3 = PathPointInch(279.31-27.5, 84.02, 0);
            ELEMENT4 = PathPointInch(279.31-27.5, 36.02, 0);

            CHARGING_MIDDLE_CONE = PathPointInch(153.93+12, 129.9, 0); // For auton starting at mid cone
            CHARGING_CENTER = PathPointInch(153.93-12, 107.85, 0);
            CHARGING_TOP_LEFT=  PathPointInch(117.16, 155.51, 0);
            CHARGING_TOP_RIGHT = PathPointInch(190.96, 155.51, 0);
            CHARGING_BOTTOM_LEFT = PathPointInch(117.16, 60.2, 0);
            CHARGING_BOTTOM_RIGHT = PathPointInch(190.96, 60.2, 0);

            // WAYPOINT_TOP = PathPointInch(190.96, 185.62, 0, 315);
            // WAYPOINT_BOTTOM = PathPointInch(190.96, 30.101, 0, 180);
            WAYPOINT_TOP = PathPointInch(190.96, 185.62, 0);
            WAYPOINT_BOTTOM = PathPointInch(190.96, 30.101, 0);
            WAYPOINT_CHARGING = new Pose2d(
                new Translation2d(
                    (ELEMENT1.getX() + CHARGING_TOP_RIGHT.getX()) / 2.0,
                    (CHARGING_TOP_RIGHT.getY() + CHARGING_BOTTOM_RIGHT.getY()) / 2.0
                ),
                Rotation2d.fromDegrees(180)
            );
        } else {
            START_TOPMOST = PathPointInch(598.41-1.5, 199.65, 180);
            START_TOP_SECOND = PathPointInch(598.41-16.5, 173.52, 180);
            START_BOTTOM_SECOND = PathPointInch(598.41-16.5, 41.67, 180);
            START_BOTTOMMOST = PathPointInch(598.41-16.5, 16.15, 180);
            START_MIDDLE_CONE = new Pose2d(
                new Translation2d(
                    START_TOPMOST.getX(),
                    ((START_TOPMOST.getY() + START_BOTTOMMOST.getY()) / 2.0) + 22.0
                ),
                Rotation2d.fromDegrees(0)
            );

            ELEMENT1 = PathPointInch(374.03+27.5, 180.02, 180);
            ELEMENT2 = PathPointInch(374.03+27.5, 132.02, 180);
            ELEMENT3 = PathPointInch(374.03+27.5, 84.02, 180);
            ELEMENT4 = PathPointInch(374.03+27.5, 36.02, 180);

            CHARGING_MIDDLE_CONE = PathPointInch(499.41-12, 129.9, 0); // For auton starting at mid cone
            CHARGING_CENTER = PathPointInch(499.41+12, 107.85, 0);
            CHARGING_TOP_LEFT = PathPointInch(462.38, 155.51, 0);
            CHARGING_TOP_RIGHT = PathPointInch(536.18, 155.51, 0);
            CHARGING_BOTTOM_LEFT = PathPointInch(462.38, 60.2, 0);
            CHARGING_BOTTOM_RIGHT = PathPointInch(536.18, 60.2, 0);

            WAYPOINT_TOP = PathPointInch(463.39, 185.62, 180.0);
            WAYPOINT_BOTTOM = PathPointInch(463.39, 30.1, 180.0);
            WAYPOINT_CHARGING = new Pose2d(
                new Translation2d(
                    (ELEMENT1.getX() + CHARGING_BOTTOM_LEFT.getX()) / 2.0,
                    (CHARGING_TOP_RIGHT.getY() + CHARGING_BOTTOM_RIGHT.getY()) / 2.0
                ),
                Rotation2d.fromDegrees(0)
            );

            
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
