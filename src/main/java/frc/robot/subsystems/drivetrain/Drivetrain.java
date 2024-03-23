// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.Optional;

import org.opencv.core.RotatedRect;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.APRILTAGS;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.sensors.AprilTagCamera;
import frc.robot.sensors.ObjectDetectionCamera;
import frc.robot.util.KnownLocations;
import frc.robot.util.PathUtils;
import frc.robot.util.SwerveUtils;
import frc.robot.util.TargetUtils;

public class Drivetrain extends SubsystemBase {

  private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
  private Pigeon2 pigeon;
  private SwerveDrivePoseEstimator odometry;
  private SwerveDriveKinematics swerveKinematics;
  private AprilTagCamera photonCam;
  private Field2d smartdashField;
  private PIDController rotationPIDController, directionalPidController;
  private PathPlannerPath pathToNote, pathToAmp;

  private ObjectDetectionCamera objectDetectionCam;
  private static final double ROTATION_P = 1.0 / 90.0, DIRECTION_P = 1 / 2.0, I = 0.0, D = 0.0;
  private final double THRESHOLD_DEGREES = 3.0;
  private final double THRESHOLD_SPEED = 0.5;
    
  // 2024 - Autotune
  private final double WHEEL_WIDTH = 23.5; // distance between front/back wheels (in inches)
  private final double WHEEL_LENGTH = 28.5; // distance between left/right wheels (in inches)

  // // 2023 - Bolt
  // private final double WHEEL_WIDTH = 27; // distance between front/back wheels (in inches)
  // private final double WHEEL_LENGTH = 27; // distance between left/right wheels (in inches)

  private Rotation2d gyroOffset = Rotation2d.fromDegrees(0); // Offset to apply to gyro for field oriented

  // Slew rate filter variables for controlling lateral acceleration
  private double currentAngularSpeed = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(SWERVE.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter angularSpeedLimiter = new SlewRateLimiter(SWERVE.ROTATIONAL_SLEW_RATE);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // configure swerve modules
    frontLeftModule = new SwerveModule(CAN.DRIVING_FRONT_LEFT, CAN.TURNING_FRONT_LEFT, -Math.PI / 2);
    frontRightModule = new SwerveModule(CAN.DRIVING_FRONT_RIGHT, CAN.TURNING_FRONT_RIGHT, 0);
    backLeftModule = new SwerveModule(CAN.DRIVING_BACK_LEFT, CAN.TURNING_BACK_LEFT, Math.PI);
    backRightModule = new SwerveModule(CAN.DRIVING_BACK_RIGHT, CAN.TURNING_BACK_RIGHT, Math.PI / 2);

    //configure gyro
    pigeon = new Pigeon2(CAN.PIGEON_DRIVETRAIN);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getYaw().setUpdateFrequency(10);

    rotationPIDController = new PIDController(ROTATION_P, I, D);
    rotationPIDController.enableContinuousInput(-180, 180);
    rotationPIDController.setTolerance(1.0);

    directionalPidController = new PIDController(DIRECTION_P, I, D);
    directionalPidController.setTolerance(0.1);

    // photonvision wrapper
    photonCam = new AprilTagCamera();
    // limelight = new Limelight();
    objectDetectionCam = new ObjectDetectionCamera();

    smartdashField = new Field2d();
    SmartDashboard.putData("Swerve Odometry", smartdashField);

    /*
    * swerve modules relative to robot center --> kinematics object --> odometry object 
    */

    double widthFromCenter = Units.inchesToMeters(WHEEL_WIDTH) / 2;
    double lengthFromCenter = Units.inchesToMeters(WHEEL_LENGTH) / 2;

    swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(lengthFromCenter, widthFromCenter),
      new Translation2d(lengthFromCenter, -widthFromCenter),
      new Translation2d(-lengthFromCenter, widthFromCenter),
      new Translation2d(-lengthFromCenter, -widthFromCenter)
    );

    // Initialize the robot odometry to the the field origin.
    // This will be updated by the selected Auton and DriveTrain.periodic()
    odometry = new SwerveDrivePoseEstimator(
      swerveKinematics, 
      getRotation(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      },
      new Pose2d(0, 0, getRotation())
    );
  }

  public PIDController getRotationalController() {
    return rotationPIDController;
  }

  public PIDController getDirectionalController() {
    return directionalPidController;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    backLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backRightModule.resetEncoders();
  }

  public void resetGyro() {
    pigeon.reset();
    gyroOffset = Rotation2d.fromDegrees(0.0);
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed) {
    drive(xSpeed, ySpeed, angularSpeed, true);
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative) {
    drive(xSpeed, ySpeed, angularSpeed, fieldRelative, false);
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(SWERVE.DIRECTION_SLEW_RATE / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        }
        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;
      
      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentAngularSpeed = angularSpeedLimiter.calculate(angularSpeed);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentAngularSpeed = angularSpeed;
    }

    double xSpeedDelivered = xSpeedCommanded * SWERVE.MAX_DIRECTION_SPEED;
    double ySpeedDelivered = ySpeedCommanded * SWERVE.MAX_DIRECTION_SPEED;
    double angularSpeedDelivered = currentAngularSpeed * SWERVE.MAX_ROTATIONAL_SPEED;

    ChassisSpeeds fieldRelativeSpeeds;

    if (fieldRelative) {
      fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, angularSpeedDelivered, getRotation(gyroOffset));
    } else {
      fieldRelativeSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, angularSpeedDelivered);
    }
    
    drive(fieldRelativeSpeeds);
  }

  public void drive(ChassisSpeeds fieldRelativeSpeeds) {
    // general swerve speeds --> speed per module
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(fieldRelativeSpeeds);

    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SWERVE.MAX_DIRECTION_SPEED);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  public void lockSwerve() {
    // set wheels into X formation
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(- Math.PI / 4)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
  }

  /** update smartdash with trajectory */
  public void setTrajectorySmartdash(Trajectory trajectory, String type) {
    smartdashField.getObject(type).setTrajectory(trajectory);
  }

  public void setPoseSmartdash(Pose2d pose, String type) {
    smartdashField.getObject(type).setPose(pose);
  }
  
  /**
   * Set the odometry object to a predetermined pose
   * No need to reset gyro as it auto-applies offset
   * 
   * Used to set initial pose from an auton trajectory
   */
  public void resetPose(Pose2d pose) {
    // Set gyro offset for field orieneted rotatioon (zero faces away from alliance station wall)
    gyroOffset = KnownLocations.getKnownLocations().ZERO_GYRO_ROTAION.plus(pose.getRotation());

    odometry.resetPosition(
    getRotation(), 
    new SwerveModulePosition[] {
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(),
      backRightModule.getPosition()
      },
    pose
    );
  }

  public ChassisSpeeds getFieldRelativSpeeds() {
    return swerveKinematics.toChassisSpeeds(new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    });
  }

  // meters/second
  //up is pos
  public double getXSpeeds() {
    return getFieldRelativSpeeds().vxMetersPerSecond;
  }

  // left is pos
  public double getYSpeeds() {
    return getFieldRelativSpeeds().vyMetersPerSecond;
  }

  /**
   * Get the robot relative rotation reported by the gyro (pigeon). Remember that this should be CCW positive.
   * @return The rotation as read from the gyro.
   */
  public Rotation2d getRotation() {
    // Note: Unlike getAngle(), getRotation2d is CCW positive.
    return pigeon.getRotation2d();
  }

  /**
   * Get the robot relative rotation reported by the gyro with an applied offset.
   * @param offset Offset to add to the gyro-supplied rotation.
   */
  public Rotation2d getRotation(Rotation2d offset) {
    return getRotation().plus(offset);
  }

  public AprilTagCamera getAprilTagCamera() {
    return this.photonCam;
  }

  public ObjectDetectionCamera getObjCam() {
    return this.objectDetectionCam;
  }

  public PathPlannerPath getPathToNote() {
    return pathToNote;
  }
  public PathPlannerPath getPathToAmp() {
    return pathToAmp;
  }

  public boolean noteInRange() {
    return objectDetectionCam.getDistanceToTarget() > 0.0 && objectDetectionCam.getDistanceToTarget() < 1.65;
  }

  public boolean isTargetPresent() {
    Optional<EstimatedRobotPose> result = photonCam.getGlobalPose();
    return result.isPresent();
  }

  public boolean isPointedAtTarget() {
    return Math.abs(getPose().getRotation().getDegrees() - TargetUtils.getTargetHeadingToFieldPosition(getPose(), FieldPosition.SPEAKER)) < THRESHOLD_DEGREES;
  }

  public boolean isNotMoving() {
    return Math.abs(getXSpeeds()) < THRESHOLD_SPEED && Math.abs(getYSpeeds()) < THRESHOLD_SPEED;
  }

  public boolean inShootingRange() {
    return TargetUtils.isInShootingZone(getPose());
  }

  public boolean objCamHasTargets() {
    return objectDetectionCam.getLatestResult().hasTargets();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      getRotation(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    });
    
    Optional<EstimatedRobotPose> result = photonCam.getGlobalPose();
    if (result.isPresent()) {
      // Uncomment the following to check camera position on robot
      // Pose3d estimatedPose = result.get().estimatedPose;
      // SmartDashboard.putNumber("Cam/Yaw", estimatedPose.getRotation().getZ());
      // SmartDashboard.putNumber("Cam/Pitch", estimatedPose.getRotation().getY());
      // SmartDashboard.putNumber("Cam/Roll", estimatedPose.getRotation().getX());
      odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
    }
    smartdashField.setRobotPose(getPose());
  
    Pose2d notePose = new Pose2d(TargetUtils.getNoteTranslation(objectDetectionCam, getPose(), objectDetectionCam.getDistanceToTarget() + Units.inchesToMeters(4.0)), TargetUtils.getTargetHeadingToClosestNote(getObjCam(), getPose()));

    setPoseSmartdash(notePose, "notepose");
    pathToNote = PathUtils.generatePath(getPose(), notePose);
    setTrajectorySmartdash(PathUtils.TrajectoryFromPath(pathToNote), "pathToNote");

    // KnownLocations knownLocations = KnownLocations.getKnownLocations();
    // pathToAmp = PathUtils.generatePath(Rotation2d.fromDegrees(-90.0), getPose(), knownLocations.AMP);
    // setTrajectorySmartdash(PathUtils.TrajectoryFromPath(pathToAmp), "pathToAmp");

    SmartDashboard.putNumber("Drivetrain/CurrentPose X", getPose().getX());
    SmartDashboard.putNumber("Drivetrain/CurrentPose Y", getPose().getY());
    SmartDashboard.putBoolean("Drivetrain/inShootingRange", inShootingRange());
    SmartDashboard.putBoolean("Drivetrain/inStageArea", TargetUtils.isInStageArea(getPose()));
    SmartDashboard.putNumber("Drivetrain/getRotation", getRotation().getDegrees());
    SmartDashboard.putBoolean("Drivetrain/pointedAtTarget", isPointedAtTarget());
    SmartDashboard.putBoolean("Drivetrain/isNotMoving", isNotMoving());
    SmartDashboard.putNumber("Drivetrain/CurrentPose Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Drivetrain/Drive Angle", getRotation().getDegrees());
    SmartDashboard.putNumber("Drivetrain/Angle to speaker without AprilTag", TargetUtils.getTargetHeadingToFieldPosition(getPose(), FieldPosition.SPEAKER));
    SmartDashboard.putNumber("Drivetrain/distanceToSpeaker", Units.metersToInches(TargetUtils.getDistanceToFieldPos(getPose(), APRILTAGS.MIDDLE_BLUE_SPEAKER)));
    SmartDashboard.putNumber("Drivetrain/New Func (angle to red)", TargetUtils.getTargetHeadingToAprilTag(getPose(), APRILTAGS.MIDDLE_RED_SPEAKER));
    SmartDashboard.putNumber("Drivetrain/Angle Offset", 0);
    SmartDashboard.putNumber("Drivetrain/Distance to closest note", objectDetectionCam.getDistanceToTarget());

  }

  public enum FieldPosition {
    AMP,
    SPEAKER;
  }
}
