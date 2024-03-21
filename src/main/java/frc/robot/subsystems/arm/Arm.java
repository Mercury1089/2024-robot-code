// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.TargetUtils;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public static final int
    ARM_PID_SLOT = 0;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0 / 10.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0;

  private static final float ARM_SOFT_LIMIT_FWD = (float) 146;

  private static final float ARM_SOFT_LIMIT_BKW = (float) 45.3;

  private static final double ANGLE_OFFSET = -3.5;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.01, //0.02,
    PEAK_OUTPUT_FORWARD = 1.0, // 0.6,
    NOMINAL_OUTPUT_REVERSE = -0.01, //-0.5,
    PEAK_OUTPUT_REVERSE = -0.6;

  public final double GEAR_RATIO = 125.0 / 1.0;
  public final double THRESHOLD_DEGREES = 0.5;

  
  private CANSparkMax armLeft;
  private CANSparkMax armRight;
  private SparkPIDController armPIDController;
  private AbsoluteEncoder armAbsoluteEncoder;
  private Drivetrain drivetrain;
  private double setPosition;

  public Arm(Drivetrain drivetrain) {
    armLeft = new CANSparkMax(CAN.ARM_LEFT, MotorType.kBrushless);
    armRight = new CANSparkMax(CAN.ARM_RIGHT, MotorType.kBrushless);

    armLeft.restoreFactoryDefaults();
    armRight.restoreFactoryDefaults();

    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);

    armLeft.setInverted(false);
    armRight.follow(armLeft, true);

    armPIDController = armLeft.getPIDController();

    armAbsoluteEncoder = armLeft.getAbsoluteEncoder(Type.kDutyCycle);

    armAbsoluteEncoder.setPositionConversionFactor(360.0);
    armPIDController.setFeedbackDevice(armAbsoluteEncoder);
    this.drivetrain = drivetrain;

    armPIDController.setPositionPIDWrappingEnabled(false);

    armLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    armLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armLeft.setSoftLimit(SoftLimitDirection.kForward, ARM_SOFT_LIMIT_FWD);
    armLeft.setSoftLimit(SoftLimitDirection.kReverse, ARM_SOFT_LIMIT_BKW);

    armPIDController.setP(ARM_NORMAL_P_VAL);
    armPIDController.setI(ARM_NORMAL_I_VAL);
    armPIDController.setD(ARM_NORMAL_D_VAL);

    setPosition = getArmPosition();

    SmartDashboard.putNumber("Arm/Position", getArmPosition());
  }
  
  public void resetEncoders() {
    armPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    armLeft.set((speedSupplier.get() * 0.5));
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

    setPosition = pos;

    armPIDController.setReference(pos, CANSparkMax.ControlType.kPosition);
  }

  public double getPosToTarget(double distance) {
    return (37.1 + (0.633 * distance) - (0.00207 * (distance * distance))) + ANGLE_OFFSET;
  }

  public double getDistanceToSpeaker() {
    return Units.metersToInches(TargetUtils.getDistanceToSpeaker(drivetrain.getPose()));
  }
  
  public double getSpeakerError() {
    return Math.abs(getArmPosition() - getPosToTarget(getDistanceToSpeaker()));
  }

  public boolean isFinishedMovingSpeaker() {
    return getSpeakerError() < THRESHOLD_DEGREES;
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
    SmartDashboard.putNumber("Arm/Encoder", getArmPosition());
    SmartDashboard.putNumber("Arm/PosToTarget", getPosToTarget(getDistanceToSpeaker()));
    SmartDashboard.putBoolean("Arm/isFinishedMoving", isFinishedMovingSpeaker());
  }
  

  public enum ArmPosition {
    AMP(ARM_SOFT_LIMIT_FWD),
    HOME(ARM_SOFT_LIMIT_BKW),
    PICKUP_FLOOR(ARM_SOFT_LIMIT_BKW);
  
    
    public final double degreePos;
      ArmPosition(double degreePos) {
        this.degreePos = degreePos;
      }
  }

}