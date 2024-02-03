// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.APRILTAGS;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.sensors.AprilTagCamera;
import frc.robot.util.SwerveUtils;

public class Drivetrain extends SubsystemBase {

  private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
  private WPI_PigeonIMU pigeon;
  private SwerveDrivePoseEstimator odometry;
  private SwerveDriveKinematics swerveKinematics;
  private AprilTagCamera photonCam;
  private Field2d smartdashField;
  private final String fieldWidgetType = "Odometry";
  
  //2024 robot
  private final double WHEEL_WIDTH = 23.5; // distance between front/back wheels (in inches)
  private final double WHEEL_LENGTH = 28.5; // distance between left/right wheels (in inches)

  // bolt
  // private final double WHEEL_WIDTH = 27; // distance between front/back wheels (in inches)
  // private final double WHEEL_LENGTH = 27; // distance between left/right wheels (in inches)

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

    // photonvision wrapper
    photonCam = new AprilTagCamera();

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

    SmartDashboard.putNumber("CurrentPose X", getPose().getX());
    SmartDashboard.putNumber("CurrentPose Y", getPose().getY());
    SmartDashboard.putNumber("CurrentPose Rotation", getPose().getRotation().getDegrees());
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

  public double getDistanceToSpeaker() {
    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    double distance = 0.0;
    if (allianceColor.isPresent()) {
      Optional<Pose3d> tagPose = (allianceColor.get() == Alliance.Blue) ? 
        photonCam.getTagPose(APRILTAGS.MIDDLE_BLUE_SPEAKER) : 
        photonCam.getTagPose(APRILTAGS.MIDDLE_RED_SPEAKER);
      if (tagPose.isPresent()) {
        distance = getPose().getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
      }
    }
    return distance;
  }

  public double getDegreesToSpeaker() {
    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    double theta = 0.0;

    if (allianceColor.isPresent()) {
      Optional<Pose3d> tagPose = (allianceColor.get() == Alliance.Blue) ? 
        photonCam.getTagPose(APRILTAGS.MIDDLE_BLUE_SPEAKER) : 
        photonCam.getTagPose(APRILTAGS.MIDDLE_RED_SPEAKER);
      if (tagPose.isPresent()) {
        double tagX = tagPose.get().toPose2d().getX();
        double roboX = getPose().getX();
        double tagY = tagPose.get().toPose2d().getY();
        double roboY = getPose().getY();
        double robotRelativeAngle = 0.0;
        double tempAngle = 0.0;
        double tagAngle = (180 / Math.PI) * Math.atan(Math.abs(roboX - tagX) / Math.abs(roboY - tagY));
        double robotDegrees = getPose().getRotation().getDegrees();
        boolean flipped = false;
        if (roboY - tagY > 0) {
          roboY -= 2 * (roboY - tagY);
          robotRelativeAngle = -1 * robotDegrees;
          flipped = true;
        } else {
          robotRelativeAngle = robotDegrees;
        }
          
        if (robotDegrees > -1 * (90 - tagAngle) && robotDegrees < 90) {
          tempAngle = (90 - robotRelativeAngle) + tagAngle;
        }  
        if (robotDegrees > 90 && robotDegrees < tagAngle ){
          tempAngle = tagAngle - (90 - robotRelativeAngle);
        }  
        if (robotDegrees > tagAngle) {
          tempAngle = (90 - robotRelativeAngle) - tagAngle;
        }
        if (robotDegrees < -1 * tagAngle) {
          tempAngle = (-180 - robotRelativeAngle) - (90 - tagAngle);
        }
        theta = flipped ? tempAngle * -1 : tempAngle;
        //double x = ((180/Math.PI) * Math.atan(Math.abs(roboX-tagX)/Math.abs(roboY-tagY)));
       // Transform2d targetTransform = new Transform2d(getPose(), tagPose.get().toPose2d());
       // theta = targetTransform.getRotation().getDegrees();
        //theta = getPose().getRotation().getDegrees() - 90.0 + (180/Math.PI) * Math.atan(Math.abs(roboX-tagX)/Math.abs(roboY-tagY));
       // theta =  ((90 - x) - headingReferenceAngle);
        //Transform2d relativePose = getPose().minus(tagPose.get().toPose2d());
        //theta = 180 - relativePose.getRotation().getDegrees();
        /*allianceColor.get() == Alliance.Blue ? 
          Math.abs(MathUtil.inputModulus(getPose().getRotation().getDegrees() - 180, -180, 180)) : 
          Math.abs(MathUtil.inputModulus(getPose().getRotation().getDegrees() + 180, -180, 180)); */
      }
    }

    return theta;
  }

  public double getDegreesToSpeakerApriltag() {
    var result = photonCam.getLatestResult();
    double yaw = 0.0;

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      
      for(int i = 0; i < targets.size() - 1; i++) {
        if (targets.get(i).getFiducialId() == APRILTAGS.MIDDLE_BLUE_SPEAKER) {
          yaw = targets.get(i).getYaw();
        }
      }
    }
    return yaw;
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

    if (fieldWidgetType.equals("Odometry")) {
      smartdashField.setRobotPose(getPose());
    } else if (fieldWidgetType.equals("photonvision")) {
      smartdashField.setRobotPose(getInitialPose());
    }

    SmartDashboard.putNumber("CurrentPose X", getPose().getX());
    SmartDashboard.putNumber("CurrentPose Y", getPose().getY());
    SmartDashboard.putNumber("CurrentPose Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Drive Angle", getPigeonRotation().getDegrees());
    SmartDashboard.putNumber("Drive Yaw", pigeon.getYaw());
    SmartDashboard.putNumber("Drive Roll", getRoll());
    SmartDashboard.putNumber("Drive Pitch", pigeon.getPitch());
    SmartDashboard.putNumber("Drive fused heading", pigeon.getFusedHeading());
    SmartDashboard.putNumber("Distance to speaker", getDistanceToSpeaker());
    SmartDashboard.putNumber("Angle to speaker without AprilTag", getDegreesToSpeaker());
    SmartDashboard.putNumber("Angle to speaker - AprilTag", getDegreesToSpeakerApriltag());
    SmartDashboard.putNumber("Robot Angle", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Tag Pose", photonCam.getTagPose(APRILTAGS.MIDDLE_BLUE_SPEAKER).get().toPose2d().getRotation().getDegrees());


    Optional<EstimatedRobotPose> result = photonCam.getGlobalPose();
    if (result.isEmpty()) {
      return;
    }
    odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);

  }
}
