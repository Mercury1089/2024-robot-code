// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Wrapper for PhotonCamera class */
public class AprilTagCamera extends PhotonCamera {

    private static final String DEFAULT_CAM_NAME = "AprilTagCamera";
    private static final double DEFAULT_CAM_X = Units.inchesToMeters(-10.5); // .5m forward of center
    private static final double DEFAULT_CAM_Y = 0.0; // centered in robot Y
    private static final double DEFAULT_CAM_Z = Units.inchesToMeters(52.25); // 52in up from center
    private final double CAMERA_HEIGHT = DEFAULT_CAM_Z; // height on robot (meters)
    private final double TARGET_HEIGHT = 0.36; // may need to change 
    private final int CAMERA_PITCH = 0; // tilt of our camera (radians)

    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator estimator;

    public AprilTagCamera() {
        super(DEFAULT_CAM_NAME);
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            fieldLayout = null;
        }
        Transform3d robotToCam = new Transform3d(
            new Translation3d(DEFAULT_CAM_X, DEFAULT_CAM_Y, DEFAULT_CAM_Z), new Rotation3d(0, 0, 0)
        );
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, this, robotToCam);
    }

    public double getDistanceToTarget() {
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, 
                Units.degreesToRadians(getPitch())
            );
            return range;
        }
        return 0.0;
    }

    public Optional<EstimatedRobotPose> getGlobalPose() {
        return estimator.update();
    }

    public double getYaw() {
        /* The yaw of the target in degrees (positive right). */
        return getLatestResult().getBestTarget().getYaw();
    }

    public double getPitch() {
        /* The pitch of the target in degrees (positive up). */
        return getLatestResult().getBestTarget().getPitch();
    }

    public double getSkew() {
        /* The skew of the target in degrees (counter-clockwise positive). */
        return getLatestResult().getBestTarget().getSkew();
    }

    public double getApriltagID() {
        return getLatestResult().getBestTarget().getFiducialId();
    }
}

