// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.Optional;
import java.util.function.Supplier;

import javax.management.relation.Relation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.APRILTAGS;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.FieldPosition;
import frc.robot.util.TargetUtils;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public static final int
    ARM_PID_SLOT = 0;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0 / 25.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0;

  private static final float ARM_SOFT_LIMIT_FWD = (float) 146;

  private static final float ARM_SOFT_LIMIT_BKW = (float) 48.5;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.01, //0.02,
    PEAK_OUTPUT_FORWARD = 1.0, // 0.6,
    NOMINAL_OUTPUT_REVERSE = -0.01, //-0.5,
    PEAK_OUTPUT_REVERSE = -0.6;

  public final double GEAR_RATIO = 125.0 / 1.0;
  public final double THRESHOLD_DEGREES = 2.0;

  
  private CANSparkFlex arm;
  private SparkPIDController armPIDController;
  private AbsoluteEncoder armAbsoluteEncoder;
  private Drivetrain drivetrain;

  public Arm(Drivetrain drivetrain) {
    arm = new CANSparkFlex(CAN.ARM_SPARKMAX, MotorType.kBrushless);
    arm.restoreFactoryDefaults();
    arm.setInverted(false);
    arm.setIdleMode(IdleMode.kBrake);
    armPIDController = arm.getPIDController();
    // armRelativeEncoder = arm.getEncoder();
    armAbsoluteEncoder = arm.getAbsoluteEncoder(Type.kDutyCycle);

    // armRelativeEncoder.setInverted(true);
    armAbsoluteEncoder.setPositionConversionFactor(360.0);
    armPIDController.setFeedbackDevice(armAbsoluteEncoder);
    this.drivetrain = drivetrain;

    armPIDController.setPositionPIDWrappingEnabled(false);
    // armPIDController.setOutputRange(NOMINAL_OUTPUT_FORWARD, PEAK_OUTPUT_FORWARD);

    arm.enableSoftLimit(SoftLimitDirection.kForward, true);
    arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
    arm.setSoftLimit(SoftLimitDirection.kForward, ARM_SOFT_LIMIT_FWD);
    arm.setSoftLimit(SoftLimitDirection.kReverse, ARM_SOFT_LIMIT_BKW);

    armPIDController.setP(ARM_NORMAL_P_VAL);
    armPIDController.setI(ARM_NORMAL_I_VAL);
    armPIDController.setD(ARM_NORMAL_D_VAL);

    SmartDashboard.putNumber("Arm/Position", getArmPosition());
  }
  
  public void resetEncoders() {
    armPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    arm.set((speedSupplier.get() * 0.5));
  }

  public void setPosition(ArmPosition pos) {
    setPosition(pos.degreePos);
  }

  public void changePos() {
    setPosition(SmartDashboard.getNumber("Arm/Position", 110.0));
  }

  public void setPosition(double pos) {
    if (pos > ARM_SOFT_LIMIT_FWD) {
      pos = ARM_SOFT_LIMIT_FWD;
    } else if (pos < ARM_SOFT_LIMIT_BKW) {
      pos = ARM_SOFT_LIMIT_BKW;
    }

    armPIDController.setReference(pos, CANSparkMax.ControlType.kPosition);
  }

  public double getPosToTarget(double distance) {
    return 37.1 + (0.633 * distance) - (0.00207 * (distance * distance));
  }

  public double getDistanceToSpeaker() {
    return Units.metersToInches(TargetUtils.getDistanceToFieldPos(drivetrain.getAprilTagCamera(), drivetrain.getPose(), APRILTAGS.MIDDLE_BLUE_SPEAKER));
  }
  
  public double getError() {
    return Math.abs(getArmPosition() - getPosToTarget(getDistanceToSpeaker()));
  }

  public boolean isFinishedMoving() {
    return getError() < THRESHOLD_DEGREES;
  }

  public boolean isAtPosition(double pos) {
    return Math.abs(getArmPosition() - pos) < THRESHOLD_DEGREES;
  }

  public boolean isAtPosition(ArmPosition pos) {
    return isAtPosition(pos.degreePos);
  }

  public double getArmPosition() {
    return armAbsoluteEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", getArmPosition());
    SmartDashboard.putNumber("arm error", getError());
    SmartDashboard.putBoolean("arm isFinished", isFinishedMoving());
    SmartDashboard.putNumber("getPos", getPosToTarget(getDistanceToSpeaker()));

    // configPID(
    //   SmartDashboard.getNumber("ARM P", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM I", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM D", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM MIN REV", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM MIN FWD", ARM_LOWER_LIMIT)
    // );

  }
  

  public enum ArmPosition {
    AMP(0.0),
    HOME(0.0),
    PICKUP_FLOOR(0.0),
    PICKUP_SOURCE(0.0);
  
    
    public final double degreePos;
      ArmPosition(double degreePos) {
        this.degreePos = degreePos;
      }
  }

}