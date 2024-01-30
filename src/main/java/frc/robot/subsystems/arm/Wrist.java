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
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.GamePiece;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Wrist extends SubsystemBase {

  private CANSparkMax wrist;
  private SparkPIDController wristPIDController;
  private AbsoluteEncoder wristAbsoluteEncoder;
  private RelativeEncoder wristRelativeEncoder;
  private Drivetrain drivetrain;

  /** Creates a new intake. */
  public Wrist(Drivetrain driveTrain) {
    wrist = new CANSparkMax(CAN.WRIST_SPARKMAX, MotorType.kBrushless);
    wrist.restoreFactoryDefaults();
    wristAbsoluteEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
    wristPIDController = wrist.getPIDController();
    
    this.drivetrain = driveTrain;
    //TODO: set encoder conversion
  }
  public void resetEncoders() {
   wristRelativeEncoder.setPosition(0);
  }

  public void setPosition(WristPosition pos) {
    wristPIDController.setReference(pos.degreePos, CANSparkMax.ControlType.kPosition);
  }
  public enum WristPosition {
    INSIDE(0.0),
    HOME(-5.0);
    
    public final double degreePos;

        WristPosition(double degreePos) {
          this.degreePos = degreePos;
        }
  }

  public double getDistanceToSpeaker() {
    return drivetrain.getDistanceToSpeaker();
  }
}