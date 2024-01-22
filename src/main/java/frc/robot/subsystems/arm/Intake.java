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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.GamePiece;

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

  private TalonSRX intake;
  private GamePieceLEDs leds;
  /** Creates a new intake. */
  public Intake(GamePieceLEDs leds) {
    this.leds = leds;

    intake = new TalonSRX(CAN.INTAKE_TALON);

    intake.configFactoryDefault();

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
  }

  public void setSpeed(IntakeSpeed intakeSpeed) {
    if (leds.getGameState() == GamePiece.CONE) {
      intake.set(ControlMode.PercentOutput, intakeSpeed.coneSpeed); 
    } else if (leds.getGameState() == GamePiece.CUBE) {
      intake.set(ControlMode.PercentOutput, intakeSpeed.cubeSpeed); 
    } else {
      intake.set(ControlMode.PercentOutput, 0.0);
    }
  }  

  public enum IntakeSpeed {
    STOP(0.0, 0.0),
    INTAKE(1.0, -1.0),
    EJECT(-1.0, 1.0);

    public final double coneSpeed;
    public final double cubeSpeed;

    private IntakeSpeed(double coneSpeed, double cubeSpeed) {
      this.coneSpeed = coneSpeed;
      this.cubeSpeed = cubeSpeed;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake current", intake.getSupplyCurrent());
    if (DriverStation.isDisabled()) {
      maxCurrent = 0.0;
    } else {
      double current = intake.getSupplyCurrent();
      if(current > maxCurrent || maxCurrentCtr > 500) {
        maxCurrent = current;
        maxCurrentCtr = 0;
      }
    }
    maxCurrentCtr++;
    SmartDashboard.putNumber("Intake max current", maxCurrent);

  }
}