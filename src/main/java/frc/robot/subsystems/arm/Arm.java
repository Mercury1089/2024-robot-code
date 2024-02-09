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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.drivetrain.Drivetrain;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public static final int
    ARM_PID_SLOT = 0;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0 / 18.0 * 1024.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0,
    ARM_NORMAL_F_VAL = 0.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.01, //0.02,
    PEAK_OUTPUT_FORWARD = 1.0, // 0.6,
    NOMINAL_OUTPUT_REVERSE = -0.01, //-0.5,
    PEAK_OUTPUT_REVERSE = -0.6;

  public final double GEAR_RATIO = 1;
  public final double THRESHOLD_DEGREES = 2.0;

  
  private CANSparkMax arm;
  private SparkPIDController armPIDController;
  private AbsoluteEncoder armAbsoluteEncoder;
  private RelativeEncoder armRelativeEncoder;
  private Drivetrain drivetrain;

  public Arm(Drivetrain drivetrain) {
    arm = new CANSparkMax(CAN.ARM_SPARKMAX, MotorType.kBrushless);
    arm.restoreFactoryDefaults();
    //armAbsoluteEncoder = arm.getAbsoluteEncoder(Type.kDutyCycle);
    armPIDController = arm.getPIDController();
    armRelativeEncoder = arm.getEncoder();
    armPIDController.setFeedbackDevice(armRelativeEncoder);
    this.drivetrain = drivetrain;

    //TODO: set encoder conversion
    
    /* 
    // Account for motor orientation.
    arm.setSensorPhase(true);
    arm.setInverted(false);

    SmartDashboard.putNumber("ARM P", ARM_NORMAL_P_VAL);
    SmartDashboard.putNumber("ARM I", ARM_NORMAL_I_VAL);
    SmartDashboard.putNumber("ARM D", ARM_NORMAL_D_VAL);
    SmartDashboard.putNumber("ARM MIN REV", NOMINAL_OUTPUT_REVERSE);
    SmartDashboard.putNumber("ARM MIN FWD", NOMINAL_OUTPUT_FORWARD);

    arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ARM_PID_SLOT, Constants.CTRE.TIMEOUT_MS);
    arm.configSelectedFeedbackCoefficient(100.0 / 200000.0);

    arm.configForwardSoftLimitThreshold(100.0);
    arm.configForwardSoftLimitEnable(true);

    arm.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);
    arm.setNeutralMode(NeutralMode.Brake);

    arm.configClearPositionOnLimitR(true, Constants.CTRE.TIMEOUT_MS);

    arm.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    arm.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    arm.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    arm.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    
    arm.configAllowableClosedloopError(ARM_PID_SLOT, 0, Constants.CTRE.TIMEOUT_MS);

    arm.config_kP(ARM_PID_SLOT, ARM_NORMAL_P_VAL, Constants.CTRE.TIMEOUT_MS);
    arm.config_kI(ARM_PID_SLOT, ARM_NORMAL_I_VAL, Constants.CTRE.TIMEOUT_MS);
    arm.config_kD(ARM_PID_SLOT, ARM_NORMAL_D_VAL, Constants.CTRE.TIMEOUT_MS);
    arm.config_kF(ARM_PID_SLOT, ARM_NORMAL_F_VAL, Constants.CTRE.TIMEOUT_MS);

    arm.selectProfileSlot(ARM_PID_SLOT, Constants.CTRE.PRIMARY_PID_LOOP);
    */
  }

  
  public void resetEncoders() {
    armRelativeEncoder.setPosition(0);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    arm.set(speedSupplier.get());
  }

  public void setPosition(ArmPosition pos) {
    armPIDController.setReference(pos.degreePos, CANSparkMax.ControlType.kPosition);
  }

  public double getDistanceToSpeaker() {
    return drivetrain.getDistanceToSpeaker();
  }
  /* 
  public double getError() {
    return arm.getClosedLoopError(ARM_PID_SLOT);
  }

  public boolean isFinishedMoving() {
    return getError() < THRESHOLD_DEGREES;
  }

  public boolean isAtPosition(ArmPosition pos) {
    return Math.abs(getArmPosition() - pos.degreePos) < THRESHOLD_DEGREES;
  }

  public double getArmPosition() {
    return arm.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", getArmPosition());
    SmartDashboard.putNumber("arm error", getError());
    SmartDashboard.putNumber("arm rev limit", arm.isRevLimitSwitchClosed());
    SmartDashboard.putBoolean("arm isFinished", isFinishedMoving());
    SmartDashboard.putBoolean("arm atPosition", isAtPosition(ArmPosition.BULLDOZER));

    // configPID(
    //   SmartDashboard.getNumber("ARM P", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM I", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM D", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM MIN REV", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM MIN FWD", ARM_LOWER_LIMIT)
    // );

  }
  */

  public enum ArmPosition {
    SPEAKER(0.0),
    AMP(0.0),
    PICKUP_FLOOR(0.0),
    PICKUP_SOURCE(0.0);
  
    
    public final double degreePos;
      ArmPosition(double degreePos) {
        this.degreePos = degreePos;
      }
  }

}