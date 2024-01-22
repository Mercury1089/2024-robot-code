package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SWERVE;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;


public class SwerveModule {
    private final CANSparkMax drivingSparkMax;
    private final CANSparkMax turningSparkMax;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkPIDController drivingPIDController;
    private final SparkPIDController turningPIDController;

    private final double DRIVING_PVAL = 0.04;
    private final double DRIVING_IVAL = 0.0;
    private final double DRIVING_DVAL = 0.0;
    private final double DRIVING_FFVAL = 1 / SWERVE.DRIVE_WHEEL_FREE_SPEED;
    private final double TURNING_PVAL = 1;
    private final double TURNING_IVAL = 0;
    private final double TURNING_DVAL = 0;
    private final double TURNING_FFVAL = 0;


    // the module position relative to robot chassis
    private double chassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveCAN, int turnCan, double chassisAngularOffset) {
        drivingSparkMax = new CANSparkMax(driveCAN, MotorType.kBrushless);
        turningSparkMax = new CANSparkMax(turnCan, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
        drivingSparkMax.restoreFactoryDefaults();
        turningSparkMax.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        drivingEncoder = drivingSparkMax.getEncoder();
        turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        drivingPIDController = drivingSparkMax.getPIDController();
        turningPIDController = turningSparkMax.getPIDController();
        drivingPIDController.setFeedbackDevice(drivingEncoder);
        turningPIDController.setFeedbackDevice(turningEncoder);

        /*
         * WPILib needs: 
         * METERS for distance
         * RADIANS for rotation
         */
        // rotations --> meters
        drivingEncoder.setPositionConversionFactor(SWERVE.METERS_CONVERSION); 
        // rpm --> m/s
        drivingEncoder.setVelocityConversionFactor(SWERVE.VELOCITY_CONVERSION);

        // deg --> radians
        turningEncoder.setPositionConversionFactor(SWERVE.RADIANS_CONVERSION);
        // dps --> rad/sec
        turningEncoder.setVelocityConversionFactor(SWERVE.RADIANS_VELOCITY_CONVERSION);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        turningEncoder.setInverted(true);

        // allow PID controller to shortcut thru 0 
        // (e.g 0 --> 350 means it only rotates -10 as it can cut thru 0)
        turningPIDController.setPositionPIDWrappingEnabled(true);
        // min = 0, max = 1 rotation in radians (2pi)
        turningPIDController.setPositionPIDWrappingMinInput(0);
        turningPIDController.setPositionPIDWrappingMaxInput(SWERVE.RADIANS_CONVERSION);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        drivingPIDController.setP(DRIVING_PVAL);
        drivingPIDController.setI(DRIVING_IVAL);
        drivingPIDController.setD(DRIVING_DVAL);
        drivingPIDController.setFF(DRIVING_FFVAL);
        drivingPIDController.setOutputRange(-1, 1);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        turningPIDController.setP(TURNING_PVAL);
        turningPIDController.setI(TURNING_IVAL);
        turningPIDController.setD(TURNING_DVAL);
        turningPIDController.setFF(TURNING_FFVAL);
        turningPIDController.setOutputRange(-1, 1);

        drivingSparkMax.setIdleMode(IdleMode.kBrake);
        turningSparkMax.setIdleMode(IdleMode.kBrake);
        drivingSparkMax.setSmartCurrentLimit(SWERVE.DRIVING_MOTOR_CURRENT_LIMIT);
        turningSparkMax.setSmartCurrentLimit(SWERVE.TURNING_MOTOR_CURRENT_LIMIT);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        drivingSparkMax.burnFlash();
        turningSparkMax.burnFlash();

        this.chassisAngularOffset = chassisAngularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }

    public void setDesiredState(SwerveModuleState speed) {

        SwerveModuleState desired = new SwerveModuleState();
        desired.speedMetersPerSecond = speed.speedMetersPerSecond;
        desired.angle = speed.angle.plus(
            Rotation2d.fromRadians(chassisAngularOffset));

        SwerveModuleState optimizedSpeed = SwerveModuleState.optimize(desired,
            new Rotation2d(turningEncoder.getPosition()));
    
        // Command driving and turning SPARKS MAX towards their respective setpoints.
        // drivingSparkMax.getPIDController().setReference(optimizedSpeed, null)
        drivingPIDController.setReference(optimizedSpeed.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningPIDController.setReference(optimizedSpeed.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

}