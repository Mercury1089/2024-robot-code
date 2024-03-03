// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Intake;

public class RobotModeLEDs extends SubsystemBase {

  private Spark blinkin;
  private RobotMode robotMode;
  private Intake intake;
  /** Creates a new GamePieceLEDs. */
  public RobotModeLEDs(Intake intake) {
    this.blinkin = new Spark(0);
    this.intake = intake;
    robotMode = RobotMode.AUTOSHOOTENABLED;
  }

  /** set by the buttons
   * - saves the game state
   * - sets the SD boolean box to color
   * - sets the physical LED
  */

  public void enableAutoShoot() {
    this.robotMode = RobotMode.AUTOSHOOTENABLED;
  }

  public void disableAutoShoot() {
    this.robotMode = RobotMode.AUTOSHOOTDISABLED;
  }

  public boolean isAutoShootEnabled() {
    return this.robotMode == RobotMode.AUTOSHOOTENABLED;
  }

  public enum LEDState {
    OFF(0.99),
    HASNOTE(0.77),
    AUTOSHOOTENABLED(0.65),
    AUTOSHOOTDISABLED(0.61);

    public final double colorValue;

    LEDState(double colorValue)  {
        this.colorValue = colorValue;
    }
  }

  public enum RobotMode {
    AUTOSHOOTENABLED,
    AUTOSHOOTDISABLED
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Color", robotMode.toString());
    SmartDashboard.putBoolean("LEDs/enableAutoShoot", isAutoShootEnabled());

    if (intake.hasNote()) {
      blinkin.set(LEDState.HASNOTE.colorValue);
    } else if (robotMode == RobotMode.AUTOSHOOTENABLED) {
      blinkin.set(LEDState.AUTOSHOOTENABLED.colorValue);
    } else if (robotMode == RobotMode.AUTOSHOOTDISABLED) {
      blinkin.set(LEDState.AUTOSHOOTDISABLED.colorValue);
    } else {
      blinkin.set(LEDState.OFF.colorValue);
    }
  }

  
}
