// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.APRILTAGS;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.auton.Autons;
import frc.robot.sensors.AprilTagCamera;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.ObjectDetectionCamera;
import frc.robot.util.PathUtils;
import frc.robot.util.SwerveUtils;
import frc.robot.util.TargetUtils;

public class Drivetrain extends SubsystemBase {

  private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
  private WPI_PigeonIMU pigeon;
  private SwerveDrivePoseEstimator odometry;
  private SwerveDriveKinematics swerveKinematics;
  private AprilTagCamera photonCam;
  private Limelight limelight;
  private Field2d smartdashField;
  private final String fieldWidgetType = "Odometry";
  private PIDController rotationPIDController;
  private PathPlannerPath pathToNote;
  private ObjectDetectionCamera objectDetectionCam;
  private Command goToNote;
  private static final double P = 1.0 / 90.0, I = 0.0, D = 0.0;
    
  //2024 robot
  // private final double WHEEL_WIDTH = 23.5; // distance between front/back wheels (in inches)
  // private final double WHEEL_LENGTH = 28.5; // distance between left/right wheels (in inches)

  // bolt
  private final double WHEEL_WIDTH = 27; // distance between front/back wheels (in inches)
  private final double WHEEL_LENGTH = 27; // distance between left/right wheels (in inches)

  public final double ROLL_WHEN_LEVEL = -1.75;

  // Slew rate filter variables for controlling lateral acceleration
  private double currentAngularSpeed = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(SWERVE.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter angularSpeedLimiter = new SlewRateLimiter(SWERVE.ROTATIONAL_SLEW_RATE);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  private Pose2d testInitialPose; 

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // configure swerve modules
    frontLeftModule = new SwerveModule(CAN.DRIVING_FRONT_LEFT, CAN.TURNING_FRONT_LEFT, -Math.PI / 2);
    frontRightModule = new SwerveModule(CAN.DRIVING_FRONT_RIGHT, CAN.TURNING_FRONT_RIGHT, 0);
    backLeftModule = new SwerveModule(CAN.DRIVING_BACK_LEFT, CAN.TURNING_BACK_LEFT, Math.PI);
    backRightModule = new SwerveModule(CAN.DRIVING_BACK_RIGHT, CAN.TURNING_BACK_RIGHT, Math.PI / 2);

    //configure gyro
    pigeon = new WPI_PigeonIMU(CAN.PIGEON_DRIVETRAIN);
    pigeon.configFactoryDefault();
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10);

    rotationPIDController = new PIDController(P, I, D);
    rotationPIDController.enableContinuousInput(-180, 180);
    rotationPIDController.setTolerance(1.0);

    // photonvision wrapper
    photonCam = new AprilTagCamera();
    limelight = new Limelight();
    objectDetectionCam = new ObjectDetectionCamera();

    smartdashField = new Field2d();
    SmartDashboard.putData("Swerve Odometry", smartdashField);

    // testInitialPose = new Pose2d(Units.inchesToMeters(54.93), Units.inchesToMeters(199.65), getPigeonRotation());
    testInitialPose = new Pose2d(0, 0, getPigeonRotation()); //  will be reset by setManualPose()

    // wpilib convienence classes
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
    odometry = new SwerveDrivePoseEstimator(
      swerveKinematics, 
      getPigeonRotation(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      },
      getInitialPose()
    );

    goToNote = new Command() {
      
    };

    SmartDashboard.putNumber("CurrentPose X", getPose().getX());
    SmartDashboard.putNumber("CurrentPose Y", getPose().getY());
    SmartDashboard.putNumber("CurrentPose Rotation", getPose().getRotation().getDegrees());
  }

  public PIDController getRotationalController() {
    return rotationPIDController;
  }

  public void resetYaw() {
    pigeon.setYaw(0);
  }

  public void calibratePigeon() {
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
  }

  public double getRoll() {
    return pigeon.getRoll();
  }

  public Pose2d getInitialPose() {
    Optional<EstimatedRobotPose> result = photonCam.getGlobalPose();
    if (result.isPresent()) {
      return result.get().estimatedPose.toPose2d();
    }
    return testInitialPose;
  }

  public boolean isTargetPresent() {
    Optional<EstimatedRobotPose> result = photonCam.getGlobalPose();
    return result.isPresent();
  }

  public void lockSwerve() {
    // set wheels into X formation
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(- Math.PI / 4)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SWERVE.MAX_DIRECTION_SPEED);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
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
  }

  public void joyDrive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative, boolean rateLimit) {
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
      fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, angularSpeedDelivered, getPigeonRotation());
    } else {
      fieldRelativeSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, angularSpeedDelivered);
    }
    
    driveFieldRelative(fieldRelativeSpeeds);
  }

  public void joyDrive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative) {
    joyDrive(xSpeed, ySpeed, angularSpeed, fieldRelative, false);
  }

  public void joyDrive(double xSpeed, double ySpeed, double angularSpeed) {
    joyDrive(xSpeed, ySpeed, angularSpeed, true);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    // general swerve speeds --> speed per module
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(fieldRelativeSpeeds);

    setModuleStates(moduleStates);
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
  public void setManualPose(Pose2d pose) {
    odometry.resetPosition(
    getPigeonRotation(), 
    new SwerveModulePosition[] {
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(),
      backRightModule.getPosition()
      },
    pose
    );
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  public ChassisSpeeds getFieldRelativSpeeds() {
    return swerveKinematics.toChassisSpeeds(new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    });
  }

  public Rotation2d getPigeonRotation() {
    /* return the pigeon's yaw as Rotation2d object */

    // Yaw is negated for field-centric in order to ensure 'true' forward of robot
    return Rotation2d.fromDegrees(-(pigeon.getAngle()));
  }

  public AprilTagCamera getAprilTagCamera() {
    return this.photonCam;
  }

  public ObjectDetectionCamera getObjCam() {
    return this.objectDetectionCam;
  }

  public Command goToNote() {
    if (objectDetectionCam.getDistanceToTarget() > 0.0 && objectDetectionCam.getDistanceToTarget() < 1.65) {
      return AutoBuilder.followPath(pathToNote);
    } else {
      return new Command() {
        
      };
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      getPigeonRotation(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    });

    Optional<EstimatedRobotPose> result = photonCam.getGlobalPose();
    if (result.isPresent()) {
      odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
    }
  
    if (fieldWidgetType.equals("Odometry")) {
      smartdashField.setRobotPose(getPose());
    } else if (fieldWidgetType.equals("photonvision")) {
      smartdashField.setRobotPose(getInitialPose());
    }

    Pose2d notePose = new Pose2d(TargetUtils.getNoteTranslation(objectDetectionCam, getPose(), objectDetectionCam.getDistanceToTarget()), new Rotation2d(TargetUtils.getTargetHeadingToClosestNote(getObjCam(), getPose()).getRadians()));

    setPoseSmartdash(notePose, "notepose");
    List<Pose2d> intermediaryNotePose = new ArrayList<>();
    
    //rotating in place
    intermediaryNotePose.add(new Pose2d(getPose().getTranslation(), new Rotation2d(TargetUtils.getTargetHeadingToClosestNote(getObjCam(), getPose()).getDegrees())));

    // check mid way
    //intermediaryNotePose.add(new Pose2d(TargetUtils.getNoteTranslation(objectDetectionCam, getPose(), objectDetectionCam.getDistanceToTarget() * 0.7), 
    //  new Rotation2d(TargetUtils.getTargetHeadingToClosestNote(getObjCam(), getPose()).getRadians())));

    pathToNote = Autons.generateSwerveTrajectory(getPose(), intermediaryNotePose, notePose);
    setTrajectorySmartdash(PathUtils.TrajectoryFromPath(pathToNote.getTrajectory(new ChassisSpeeds(), getPose().getRotation())), "pathToNote");

    SmartDashboard.putNumber("CurrentPose X", getPose().getX());
    SmartDashboard.putNumber("CurrentPose Y", getPose().getY());
    SmartDashboard.putNumber("CurrentPose Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Drive Angle", getPigeonRotation().getDegrees());
    SmartDashboard.putNumber("Drive Yaw", pigeon.getYaw());
    SmartDashboard.putNumber("Drive Roll", getRoll());
    SmartDashboard.putNumber("Drive Pitch", pigeon.getPitch());
    SmartDashboard.putNumber("Drive fused heading", pigeon.getFusedHeading());
    // SmartDashboard.putNumber("Distance to speaker", getDistanceToSpeaker());
    SmartDashboard.putNumber("Angle to speaker without AprilTag", TargetUtils.getTargetHeadingToFieldPosition(photonCam, getPose(), FieldPosition.SPEAKER));
    SmartDashboard.putNumber("Angle Offset", 0);
    // SmartDashboard.putNumber("Angle to speaker - AprilTag", getDegreesToSpeakerApriltag());
    // SmartDashboard.putNumber("X to closest note", getClosestNoteX());
    // SmartDashboard.putNumber("Y to closest note", getClosestNoteY());
    // SmartDashboard.putNumber("Angle to closest note", objectDetectionCamera.getYaw());
    SmartDashboard.putNumber("Distance to closest note", objectDetectionCam.getDistanceToTarget());
    // SmartDashboard.putNumber("Target Heading to note", getTargetHeadingToClosestNote());
    SmartDashboard.putNumber("Robot Angle", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Tag Pose Angle", photonCam.getTagPose(APRILTAGS.MIDDLE_BLUE_SPEAKER).get().toPose2d().getRotation().getDegrees());
    SmartDashboard.putNumber("Tag Pose X", photonCam.getTagPose(APRILTAGS.MIDDLE_BLUE_SPEAKER).get().toPose2d().getTranslation().getX());

  }

  public enum FieldPosition {
    AMP,
    SPEAKER;
  }
}
