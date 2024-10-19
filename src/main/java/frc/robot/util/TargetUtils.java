package frc.robot.util;

import java.util.Optional;

import javax.management.openmbean.OpenType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.APRILTAGS;
import frc.robot.commands.Autons;
import frc.robot.commands.Autons.AutonTypes;
import frc.robot.sensors.ObjectDetectionCamera;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.FieldPosition;

public class TargetUtils {

    public static double getDistanceToFieldPos(Pose2d robotPose, int apriltag) {
        double distance = 0.0;
        Optional<Pose3d> tagPose = KnownLocations.getFieldLayout().getTagPose(apriltag);

        if (tagPose.isPresent()) {
            distance = robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
        }

        return distance;
    }

    public static double getDistanceToSpeaker(Pose2d robotPose) {
        Alliance alliance = KnownLocations.getKnownLocations().alliance;
        if (alliance == Alliance.Blue) {
            return getDistanceToFieldPos(robotPose, APRILTAGS.MIDDLE_BLUE_SPEAKER);
        } else {
            return getDistanceToFieldPos(robotPose, APRILTAGS.MIDDLE_RED_SPEAKER);
        }
    }

    // Following is based off https://www.chiefdelphi.com/t/is-there-a-builtin-function-to-find-the-angle-needed-to-get-one-pose2d-to-face-another-pose2d/455972
    public static double getTargetHeadingToAprilTag(Pose2d robotPose, int tagId) {
        double heading = 0.0;
        Optional<Pose3d> tagPose = KnownLocations.getFieldLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
            Translation2d tagPoint = tagPose.get().getTranslation().toTranslation2d();
            Rotation2d targetRotation = tagPoint.minus(robotPose.getTranslation()).getAngle();
            heading = targetRotation.rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
        }
        return heading;
    }

    public static double getTargetHeadingToPoint(Pose2d robotPose, Translation2d point) {
        Rotation2d targetRotation = point.minus(robotPose.getTranslation()).getAngle();
        return  targetRotation.rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
    }

    public static double getTargetHeadingToFieldPosition(Pose2d robotPose, FieldPosition fieldPos) {
        double heading = 0.0;
        Alliance alliance = KnownLocations.getKnownLocations().alliance;
        
        if (fieldPos == FieldPosition.SPEAKER) {
            int apriltag = (alliance == Alliance.Blue) ?
                APRILTAGS.MIDDLE_BLUE_SPEAKER :
                APRILTAGS.MIDDLE_RED_SPEAKER;
            heading = getTargetHeadingToAprilTag(robotPose, apriltag);
        } else if (fieldPos == FieldPosition.AMP) {
            int apriltag = (alliance == Alliance.Blue) ?
                APRILTAGS.BLUE_AMP :
                APRILTAGS.RED_AMP;
            heading = getTargetHeadingToAprilTag(robotPose, apriltag);       
        }
        //should never get here
        return heading;
    }

    public static int getAmpTag() {
        return KnownLocations.getKnownLocations().alliance == Alliance.Blue ?
            APRILTAGS.BLUE_AMP :
            APRILTAGS.RED_AMP;
    }

    public static Rotation2d getTargetHeadingToClosestNote(ObjectDetectionCamera objCam, Pose2d robotPose) {
        Rotation2d targetRotation = Rotation2d.fromDegrees(-objCam.getYaw());
        return objCam.getYaw() != 0.0 ?
            robotPose.rotateBy(targetRotation).getRotation() :
            robotPose.getRotation();
    }

   public static Translation2d getNoteTranslation(ObjectDetectionCamera objCam, Pose2d robotPose, double distance) {
        return new Translation2d(distance + Units.inchesToMeters(5.0), getTargetHeadingToClosestNote(objCam, robotPose)).plus(robotPose.getTranslation());
   }

   public static boolean isInWing(Pose2d pose) {
        KnownLocations knownLocations = KnownLocations.getKnownLocations();
        Alliance alliance = knownLocations.alliance;

        return alliance == Alliance.Blue ?
            Units.metersToInches(pose.getX()) < (Units.metersToInches(knownLocations.WING_NOTE_TOP.getX()) /*+ 50.0 */):
            Units.metersToInches(pose.getX()) > (Units.metersToInches(knownLocations.WING_NOTE_TOP.getX()) /*- 50.0 */);
   }

   public static boolean isInAmpZone(Pose2d pose) {
    KnownLocations knownLocations = KnownLocations.getKnownLocations();
    return knownLocations.alliance == Alliance.Blue ? 
      pose.getY() > knownLocations.WING_NOTE_BOTTOM.getY() && Units.metersToInches(pose.getX()) < (knownLocations.WING_LINE_BLUE + 24):
      pose.getY() > knownLocations.WING_NOTE_BOTTOM.getY() && Units.metersToInches(pose.getX()) > (knownLocations.WING_LINE_RED - 24);
   }

   public static boolean isInCenterZone(Pose2d pose) {
    KnownLocations knownLocations = KnownLocations.getKnownLocations();
    return Units.metersToInches(pose.getX()) > knownLocations.WING_LINE_BLUE && Units.metersToInches(pose.getX()) < knownLocations.WING_LINE_RED;
   }

   public static boolean ampShotCheck(Pose2d pose) {
    int ampTag = KnownLocations.getKnownLocations().alliance == Alliance.Blue ? APRILTAGS.BLUE_AMP : APRILTAGS.RED_AMP;
    return /*(Math.abs(KnownLocations.getFieldLayout().getTagPose(ampTag).get().getX() - pose.getX()) < 0.05) && 
      (Math.abs(pose.getRotation().getDegrees() + 90.0) < 0.75) && */
      (getDistanceToFieldPos(pose, ampTag) < 2.0);
   }

   // TODO: Create KnownLocations for the X values in this method
   public static boolean isInShootingZone(Pose2d pose) {
       boolean inShootingZone = false;
       Alliance alliance = KnownLocations.getKnownLocations().alliance;

       if (!isInStageArea(pose)) {
           if (alliance == Alliance.Blue) {
               inShootingZone = Units.metersToInches(pose.getX()) < 200.0;
           } else if (alliance == Alliance.Red) {
               inShootingZone = Units.metersToInches(pose.getX()) > 450.0;
           }
       }
       return inShootingZone;
   }

   // TODO: Create KnownLocations for the X/Y values in this method
   public static boolean isInStageArea(Pose2d pose) {
       boolean inStageArea = false;
       Alliance alliance = KnownLocations.getKnownLocations().alliance;

       double x = Units.metersToInches(pose.getX());
       double y = Units.metersToInches(pose.getY());
       if (alliance == Alliance.Blue) {
           inStageArea = (y > ((-0.57735 * x) + 227.16483)) &&
                   (y < ((0.57735 * x) + 96.85518)) &&
                   (x < 230.0);
       } else if (alliance == Alliance.Red) {
           inStageArea = (y < ((-0.57735 * x) + 473.10859)) &&
                   (y > ((0.57735 * x) - 149.10859)) &&
                   (x > 420.0);
       }

       return inStageArea;
   }

   public static Optional<Rotation2d> getRotationTargetOverride(Autons auto, Drivetrain drivetrain, Intake intake, Arm arm){
    // var result = drivetrain.getObjCam().getLatestResult();   <- Don't need this anymore
    if (auto.getAutonType() == AutonTypes.CENTER_LINE_NOTES && TargetUtils.isInWing(drivetrain.getPose())) {
        return Optional.empty();
    } else if (/*result.hasTargets() && */ !intake.hasNote() && auto.noteInRange()) { // added auto.noteInRange()
        return Optional.of(getTargetHeadingToClosestNote(drivetrain.getObjCam(), drivetrain.getPose()));
    } else if (intake.hasNote()) {
        return Optional.of(Rotation2d.fromDegrees(getTargetHeadingToFieldPosition(drivetrain.getPose(), FieldPosition.SPEAKER)));
    } else {
        return Optional.empty();
    }
}

}
