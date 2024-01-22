// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.sensors.AprilTagCamera;

public class Drivetrain extends SubsystemBase {

  private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
  private WPI_PigeonIMU pigeon;
  private SwerveDrivePoseEstimator odometry;
  private SwerveDriveKinematics swerveKinematics;
  private AprilTagCamera photonCam;
  private Field2d smartdashField;
  private final String fieldWidgetType = "Odometry";
  
  private final double WHEEL_WIDTH = 27; // distance between front/back wheels (in inches)
  private final double WHEEL_LENGTH = 27; // distance between left/right wheels (in inches)
  public final double ROLL_WHEN_LEVEL = -1.75;

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
      getInitialPose());

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

  public void joyDrive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative) {
    xSpeed *= SWERVE.MAX_DIRECTION_SPEED;
    ySpeed *= SWERVE.MAX_DIRECTION_SPEED;
    angularSpeed *= SWERVE.MAX_ROTATIONAL_SPEED;

    ChassisSpeeds fieldRelativeSpeeds;

    if (fieldRelative) {
      fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angularSpeed, getPigeonRotation());
    } else {
      fieldRelativeSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angularSpeed);
    }
    
    // general swerve speeds --> speed per module
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(fieldRelativeSpeeds);

    setModuleStates(moduleStates);
  }

  public void joyDrive(double xSpeed, double ySpeed, double angularSpeed) {
    joyDrive(xSpeed, ySpeed, angularSpeed, true);
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


  public Rotation2d getPigeonRotation() {
    /* return the pigeon's yaw as Rotation2d object */

    // Yaw is negated for field-centric in order to ensure 'true' forward of robot
    return Rotation2d.fromDegrees(-(pigeon.getAngle()));
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


    Optional<EstimatedRobotPose> result = photonCam.getGlobalPose();
    if (result.isEmpty()) {
      return;
    }
    odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);

  }
}
