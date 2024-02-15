// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.io.IOException;
import java.util.Optional;

import javax.imageio.IIOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.APRILTAGS;
import frc.robot.subsystems.drivetrain.Drivetrain;

/** Wrapper for PhotonCamera class */
public class ObjectDetectionCamera extends PhotonCamera {

    //TODO: UPDATE CAM SETTINGS FOR NEW ROBOT
    private static final String DEFAULT_CAM_NAME = "ObjectDetectionCam";
    private final double CAMERA_HEIGHT_METERS = 0.0457; // height on robot (meters)
    private final double TARGET_HEIGHT_METERS = 0.0254; // may need to change 
    private final int CAMERA_PITCH_RADIANS = 0; // tilt of our camera (radians)

    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator estimator;

    public ObjectDetectionCamera() {
        super(DEFAULT_CAM_NAME);
    }

    public double getYaw() {
        /* The yaw of the target in degrees (positive right). */
        return getLatestResult().hasTargets() ? 
            getLatestResult().getBestTarget().getYaw() :
            0.0;
    }

    public double getPitch() {
        /* The pitch of the target in degrees (positive up). */
        return getLatestResult().getBestTarget().getPitch();
    }

    public double getSkew() {
        /* The skew of the target in degrees (counter-clockwise positive). */
        return getLatestResult().getBestTarget().getSkew();
    }

    public double getDistanceToNote() {
        double range = 0.0;
        if (getLatestResult().hasTargets()) {
            range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(getLatestResult().getBestTarget().getPitch()));
        }
        return range;
    }
}