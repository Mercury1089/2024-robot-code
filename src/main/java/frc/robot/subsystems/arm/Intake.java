// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BREAKBEAM;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.RobotModeLEDs;

public class Intake extends SubsystemBase {

  public static final int INTAKE_PID_SLOT = 0;

  private static final double
    intake_NORMAL_P_VAL = 1.0, // / 25.0 * 1024.0,
    intake_NORMAL_I_VAL = 0.0,
    intake_NORMAL_D_VAL = 0.0,
    intake_NORMAL_F_VAL = 0.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.02,
    NOMINAL_OUTPUT_REVERSE = -0.02,
    PEAK_OUTPUT_FORWARD = 1,
    PEAK_OUTPUT_REVERSE = -1;

  private double maxCurrent = 0.0;
  private int maxCurrentCtr = 0;

  private CANSparkFlex intake;
  private final int INTAKE_BREAKBEAM = BREAKBEAM.INTAKE_BREAKBEAM;
  private DigitalInput intakeBreakBeam;

  /** Creates a new intake. */
  public Intake() {

    intake = new CANSparkFlex(CAN.INTAKE_SPARKFLEX, MotorType.kBrushless);

    intake.restoreFactoryDefaults();

    intakeBreakBeam = new DigitalInput(INTAKE_BREAKBEAM);

    /* 
    // intake.setSensorPhase(false);
    intake.setInverted(true);
   
    intake.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    intake.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    intake.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    intake.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    intake.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    intake.setNeutralMode(NeutralMode.Brake);

    intake.config_kP(INTAKE_PID_SLOT, intake_NORMAL_P_VAL, Constants.CTRE.TIMEOUT_MS);
    intake.config_kI(INTAKE_PID_SLOT, intake_NORMAL_I_VAL, Constants.CTRE.TIMEOUT_MS);
    intake.config_kD(INTAKE_PID_SLOT, intake_NORMAL_D_VAL, Constants.CTRE.TIMEOUT_MS);
    intake.config_kF(INTAKE_PID_SLOT, intake_NORMAL_F_VAL, Constants.CTRE.TIMEOUT_MS);
    */
  }

  public boolean hasNote() {
    return !intakeBreakBeam.get();
  }

  public void setSpeed(IntakeSpeed intakeSpeed) {
    intake.set(intakeSpeed.speed);
  }

  public enum IntakeSpeed {
    INTAKE(1.0),
    OUTTAKE(-1.0),
    SHOOT(1.0),
    STOP(0.0);
  
    
    public final double speed;
      IntakeSpeed(double speed) {
        this.speed = speed;
      }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/hasNote", hasNote());
  }
}