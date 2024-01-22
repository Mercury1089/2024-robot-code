// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import java.util.Optional;

import com.pathplanner.lib.PathPoint;

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
    public static final PathPoint
        DO_NOTHING = PathPointInch(0, 0, 0, 0);

    // auton starts
    public final PathPoint 
        START_TOPMOST,
        START_TOP_SECOND,
        START_BOTTOM_SECOND,
        START_BOTTOMMOST,
        START_MIDDLE_CONE;
    
    // elements
    public final PathPoint
        ELEMENT1,
        ELEMENT2,
        ELEMENT3,
        ELEMENT4;
    
    //charging station
    public final PathPoint
        CHARGING_MIDDLE_CONE,
        CHARGING_CENTER,
        CHARGING_TOP_LEFT,
        CHARGING_TOP_RIGHT,
        CHARGING_BOTTOM_LEFT,
        CHARGING_BOTTOM_RIGHT;

    public final PathPoint
        WAYPOINT_TOP,
        WAYPOINT_BOTTOM,
        WAYPOINT_CHARGING;
        // WAYPOINT_CHARGING_BACK;
    
        
    public Optional<Alliance> allianceColor = DriverStation.getAlliance();

    public KnownLocations() {

        allianceColor = DriverStation.getAlliance();

        if ( allianceColor.isPresent() && allianceColor.get() == Alliance.Blue) {
            START_TOPMOST = PathPointInch(54.93+16.5, 199.65, 0, 180);
            START_TOP_SECOND = PathPointInch(54.93+16.5, 173.52, 0, 180);
            START_BOTTOM_SECOND = PathPointInch(54.93+16.5, 41.67, 0, 180);
            START_BOTTOMMOST = PathPointInch(54.93+16.5, 16.15, 0, -180);
            START_MIDDLE_CONE = PathPointInch(54.93+16.5, 129.9, 0, 180);
            // START_MIDDLE_CONE = new PathPoint(
            //     new Translation2d(
            //         START_TOPMOST.position.getX(),
            //         ((START_TOPMOST.position.getY() + START_BOTTOMMOST.position.getY()) / 2.0) + 22.0
            //     ),
            //     Rotation2d.fromDegrees(0),
            //     Rotation2d.fromDegrees(180)
            // );

            ELEMENT1 = PathPointInch(279.31-27.5, 180.02, 0, 0);
            ELEMENT2 = PathPointInch(279.31-27.5, 132.02, 0, 0);
            ELEMENT3 = PathPointInch(279.31-27.5, 84.02, 0, 0);
            ELEMENT4 = PathPointInch(279.31-27.5, 36.02, 0, 0);

            CHARGING_MIDDLE_CONE = PathPointInch(153.93+12, 129.9, 0, 0); // For auton starting at mid cone
            CHARGING_CENTER = PathPointInch(153.93-12, 107.85, 0, 0);
            CHARGING_TOP_LEFT=  PathPointInch(117.16, 155.51, 0, 0);
            CHARGING_TOP_RIGHT = PathPointInch(190.96, 155.51, 0, 0);
            CHARGING_BOTTOM_LEFT = PathPointInch(117.16, 60.2, 0, 0);
            CHARGING_BOTTOM_RIGHT = PathPointInch(190.96, 60.2, 0, 0);

            // WAYPOINT_TOP = PathPointInch(190.96, 185.62, 0, 315);
            // WAYPOINT_BOTTOM = PathPointInch(190.96, 30.101, 0, 180);
            WAYPOINT_TOP = PathPointInch(190.96, 185.62, 0, 40.0);
            WAYPOINT_BOTTOM = PathPointInch(190.96, 30.101, 0, 40.0);
            WAYPOINT_CHARGING = new PathPoint(
                new Translation2d(
                    (ELEMENT1.position.getX() + CHARGING_TOP_RIGHT.position.getX()) / 2.0,
                    (CHARGING_TOP_RIGHT.position.getY() + CHARGING_BOTTOM_RIGHT.position.getY()) / 2.0
                ),
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(0)
            );
            // WAYPOINT_CHARGING_BACK = new PathPoint(
            //     new Translation2d(
            //         ((ELEMENT1.position.getX() + CHARGING_TOP_RIGHT.position.getX()) / 2.0)
            //         + () ,
            //         (CHARGING_TOP_RIGHT.position.getY() + CHARGING_BOTTOM_RIGHT.position.getY()) / 2.0
            //     ),
            //     null,
            //     null);
        } else {
            START_TOPMOST = PathPointInch(598.41-1.5, 199.65, 180, 0);
            START_TOP_SECOND = PathPointInch(598.41-16.5, 173.52, 180, 0);
            START_BOTTOM_SECOND = PathPointInch(598.41-16.5, 41.67, 180, 0);
            START_BOTTOMMOST = PathPointInch(598.41-16.5, 16.15, 180, 0);
            START_MIDDLE_CONE = new PathPoint(
                new Translation2d(
                    START_TOPMOST.position.getX(),
                    ((START_TOPMOST.position.getY() + START_BOTTOMMOST.position.getY()) / 2.0) + 22.0
                ),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(180)
            );

            ELEMENT1 = PathPointInch(374.03+27.5, 180.02, 180, 180);
            ELEMENT2 = PathPointInch(374.03+27.5, 132.02, 180, 180);
            ELEMENT3 = PathPointInch(374.03+27.5, 84.02, 180, 180);
            ELEMENT4 = PathPointInch(374.03+27.5, 36.02, 180, 180);

            CHARGING_MIDDLE_CONE = PathPointInch(499.41-12, 129.9, 0, 0); // For auton starting at mid cone
            CHARGING_CENTER = PathPointInch(499.41+12, 107.85, 0, 180);
            CHARGING_TOP_LEFT = PathPointInch(462.38, 155.51, 0, 180);
            CHARGING_TOP_RIGHT = PathPointInch(536.18, 155.51, 0, 180);
            CHARGING_BOTTOM_LEFT = PathPointInch(462.38, 60.2, 0, 180);
            CHARGING_BOTTOM_RIGHT = PathPointInch(536.18, 60.2, 0, 180);

            // WAYPOINT_TOP = PathPointInch(463.39, 185.62, 0, 180);
            // WAYPOINT_BOTTOM = PathPointInch(463.39, 30.1, 0, 180);
            // WAYPOINT_CHARGING = new PathPoint(
            //     new Translation2d(
            //         (ELEMENT1.position.getX() + CHARGING_BOTTOM_LEFT.position.getX()) / 2.0,
            //         (CHARGING_TOP_RIGHT.position.getY() + CHARGING_BOTTOM_RIGHT.position.getY()) / 2.0
            //     ),
            //     Rotation2d.fromDegrees(0),
            //     Rotation2d.fromDegrees(180)
            // );
            WAYPOINT_TOP = PathPointInch(463.39, 185.62, 180.0, 140.0);
            WAYPOINT_BOTTOM = PathPointInch(463.39, 30.1, 180.0, 140.0);
            WAYPOINT_CHARGING = new PathPoint(
                new Translation2d(
                    (ELEMENT1.position.getX() + CHARGING_BOTTOM_LEFT.position.getX()) / 2.0,
                    (CHARGING_TOP_RIGHT.position.getY() + CHARGING_BOTTOM_RIGHT.position.getY()) / 2.0
                ),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(180)
            );

            
        }
    }

    /** Convenience method to create PathPoint from inches */
    private static PathPoint PathPointInch(double xInches, double yInches, double headingDegrees, double rotationDegrees) {
        return new PathPoint(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)), Rotation2d.fromDegrees(headingDegrees), Rotation2d.fromDegrees(rotationDegrees));
    }

    /** Convenience method to create waypoint excluding holonomic rotation */
    private static PathPoint PathWayPoint(double xInches, double yInches, double headingDegrees) {
        return new PathPoint(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)), Rotation2d.fromDegrees(headingDegrees), null);
    }

}
