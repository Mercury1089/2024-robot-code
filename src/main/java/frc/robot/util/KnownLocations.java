// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.UncheckedIOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Absolute (X, Y) of certain field locations
 * (thanks design :)
 */
public class KnownLocations {

    // The KnownLocations singleton instance returned by getKnownLocaitons()
    private static KnownLocations knownLocations = null;
    private static AprilTagFieldLayout fieldLayout = null;

    // The alliance of the current KnownLocations
    public final Alliance alliance;

    // do nothing auton
    public final Pose2d DO_NOTHING = PathPointInch(0, 0, 0);

    // looking from its alliance side
    public final Pose2d
            START_TOPMOST,
            START_MIDDLE,
            START_BOTTOMMOST;

    // notes -> NOTE_WING_1 lines up w/NOTE_CENTER_1 from the top
    // starting positions are looking from blue side of field
    public final Pose2d
            WING_NOTE_TOP,
            WING_NOTE_MIDDLE,
            WING_NOTE_BOTTOM,
            INTERMEDIARY_NOTE_TOP,
            INTERMEDIARY_NOTE_MIDDLE,
            INTERMEDIARY_NOTE_BOTTOM,
            CENTER_NOTE_1,
            CENTER_NOTE_2,
            CENTER_NOTE_3,
            CENTER_NOTE_4,
            CENTER_NOTE_5,
            LEAVE,
            AMP;

    /**
     * Load the field layout for the current year (currently CRESCENDO).
     * 
     * <p>
     * This is a singleton instance of the field layout for any class that needs it.
     * 
     * @return The current AprilTagFieldLayout or null if there is an UncheckedIOException loading the file.
     *         A warning is reported on the Driver Station if an exception is encountered
     */
    synchronized public static AprilTagFieldLayout getFieldLayout() {
        if (fieldLayout == null) {
            try {
                fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
            } catch (UncheckedIOException e) {
                DriverStation.reportWarning("Failed to load AprilTagFieldLayout: " + e.getMessage(), true);
                fieldLayout = null;
            }
        }
        return fieldLayout;
    }

    /**
     * Get an updated KnownLocations based on the current alliance.
     * 
     * <p>
     * This will generate a new KnownLocations instance based on the current
     * alliance.
     * Do not save the returned instance. Always call this method to get the latest
     * copy.
     * 
     * @return The current KnownLocations instance. Never save this instance as it
     *         can change.
     */
    synchronized public static KnownLocations getKnownLocations() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (knownLocations == null || knownLocations.alliance != alliance) {
            knownLocations = new KnownLocations(alliance);
        }
        return knownLocations;
    }

    private KnownLocations(Alliance alliance) {
        this.alliance = alliance;

        if (alliance == Alliance.Blue) {

            WING_NOTE_TOP = PathPointInch(114.261 - 18.0, 276, 0);
            WING_NOTE_MIDDLE = PathPointInch(114.261, 219, 0);
            WING_NOTE_BOTTOM = PathPointInch(114.261, 162, 0);

            LEAVE = PathPointInch(114.261, 162 - 50, 0);

            AMP = PathPointInch(72.5, 303.0, 90.0);

            INTERMEDIARY_NOTE_TOP = PathPointInch(84.261 - 14, 276, 0);
            INTERMEDIARY_NOTE_MIDDLE = PathPointInch(84.261 - 14, 219, 0);
            INTERMEDIARY_NOTE_BOTTOM = PathPointInch(84.261 - 14, 162, 0);

            CENTER_NOTE_1 = PathPointInch(325.862, 30, 0);
            CENTER_NOTE_2 = PathPointInch(325.862, 96, 0);
            CENTER_NOTE_3 = PathPointInch(325.862, 162, 0);
            CENTER_NOTE_4 = PathPointInch(325.862, 228, 0);
            CENTER_NOTE_5 = PathPointInch(325.862, 294, 0);

            START_TOPMOST = PathPointInch(24.741, 268.182, 60);
            START_MIDDLE = PathPointInch(55.43, 218.777885, 0);
            START_BOTTOMMOST = PathPointInch(24.741, 169.374, -60);

        } else {

            WING_NOTE_TOP = PathPointInch(537.464, 276, 180);
            WING_NOTE_MIDDLE = PathPointInch(537.464, 219, 180);
            WING_NOTE_BOTTOM = PathPointInch(537.464, 162, 180);

            LEAVE = PathPointInch(537.464, 162 - 50, 180);

            AMP = PathPointInch(578.77, 303.0, 90.0);

            INTERMEDIARY_NOTE_TOP = PathPointInch(567.464 + 14, 276, 180);
            INTERMEDIARY_NOTE_MIDDLE = PathPointInch(567.464 + 14, 219, 180);
            INTERMEDIARY_NOTE_BOTTOM = PathPointInch(567.464 + 14, 162, 180);

            CENTER_NOTE_1 = PathPointInch(325.862, 30, 180);
            CENTER_NOTE_2 = PathPointInch(325.862, 96, 180);
            CENTER_NOTE_3 = PathPointInch(325.862, 162, 180);
            CENTER_NOTE_4 = PathPointInch(325.862, 228, 180);
            CENTER_NOTE_5 = PathPointInch(325.862, 294, 180);

            START_TOPMOST = PathPointInch(626.983, 266.305, 120);
            START_MIDDLE = PathPointInch(596.295, 218.777, 0);
            START_BOTTOMMOST = PathPointInch(623.735, 169.372, -120);

        }
    }

    /** Convenience method to create PathPoint from inches */
    private static Pose2d PathPointInch(double xInches, double yInches, double headingDegrees) {
        return new Pose2d(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)),
                Rotation2d.fromDegrees(headingDegrees));
    }
}