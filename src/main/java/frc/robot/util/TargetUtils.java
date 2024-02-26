package frc.robot.util;

import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.APRILTAGS;
import frc.robot.sensors.AprilTagCamera;
import frc.robot.sensors.ObjectDetectionCamera;
import frc.robot.subsystems.drivetrain.Drivetrain.FieldPosition;

public class TargetUtils {

    public static double getDistanceToFieldPos(AprilTagCamera photonCam, Pose2d robotPose, int apriltag) {
        double distance = 0.0;
        Optional<Pose3d> tagPose = photonCam.getTagPose(apriltag);

        if (tagPose.isPresent()) {
            distance = robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
        }

        return distance;
    }

    // TODO: Try this and compare getTargetHeadingToFieldPosition. targetRotation man need to be added to robotPose.getRoation()
    // Following is based off https://www.chiefdelphi.com/t/is-there-a-builtin-function-to-find-the-angle-needed-to-get-one-pose2d-to-face-another-pose2d/455972
    public static double getTargetHeadingToAprilTag(AprilTagCamera photonCam, Pose2d robotPose, int tagId) {
        double heading = 0.0;
        Optional<Pose3d> tagPose = photonCam.getTagPose(tagId);
        if (tagPose.isPresent()) {
            Translation2d tagPoint = tagPose.get().getTranslation().toTranslation2d();
            Rotation2d targetRotation = tagPoint.minus(robotPose.getTranslation()).getAngle();
            heading = targetRotation.rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
        }
        return heading;
    }

    public static double getTargetHeadingToFieldPosition(AprilTagCamera photonCam, Pose2d robotPose, FieldPosition fieldPos) {
        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        Optional<Pose3d> tagPose = Optional.empty();
        
        if (fieldPos == FieldPosition.SPEAKER) {
            int apriltag = (allianceColor.get() == Alliance.Blue) ?
                APRILTAGS.MIDDLE_BLUE_SPEAKER :
                APRILTAGS.MIDDLE_RED_SPEAKER;
            tagPose = photonCam.getTagPose(apriltag);
            
            return robotPose.getY() < tagPose.get().getTranslation().getY() ? 
                -Math.acos((robotPose.getTranslation().getX() - tagPose.get().getTranslation().getX()) / getDistanceToFieldPos(photonCam, robotPose, apriltag)) * (180 / Math.PI) :
                Math.acos((robotPose.getTranslation().getX() - tagPose.get().getTranslation().getX()) / getDistanceToFieldPos(photonCam, robotPose, apriltag)) * (180 / Math.PI);
        } else if (fieldPos == FieldPosition.AMP) {
            int apriltag = (allianceColor.get() == Alliance.Blue) ?
                APRILTAGS.BLUE_AMP :
                APRILTAGS.RED_AMP;
            tagPose = photonCam.getTagPose(apriltag);

            return robotPose.getX() < tagPose.get().getTranslation().getX() ? 
                Math.asin((tagPose.get().getTranslation().getY() - robotPose.getTranslation().getY()) / getDistanceToFieldPos(photonCam, robotPose, apriltag)) * (180 / Math.PI) - 180 :
                -Math.asin((tagPose.get().getTranslation().getY() - robotPose.getTranslation().getY()) / getDistanceToFieldPos(photonCam, robotPose, apriltag)) * (180 / Math.PI);
        }

        //should never get here
        return 0.0;
    }

    public static Rotation2d getTargetHeadingToClosestNote(ObjectDetectionCamera objCam, Pose2d robotPose) {
        Rotation2d targetRotation = Rotation2d.fromDegrees(-objCam.getYaw());
        return objCam.getYaw() != 0.0 ?
            robotPose.rotateBy(targetRotation).getRotation() :
            robotPose.getRotation();
    }

   public static Translation2d getNoteTranslation(ObjectDetectionCamera objCam, Pose2d robotPose, double distance) {
        return new Translation2d(distance, getTargetHeadingToClosestNote(objCam, robotPose)).plus(robotPose.getTranslation());
   }
}
