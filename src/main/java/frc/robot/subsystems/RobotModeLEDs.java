// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotModeLEDs extends SubsystemBase {

  private Spark blinkin;
  private LEDState robotMode;
  /** Creates a new GamePieceLEDs. */
  public RobotModeLEDs() {
    this.blinkin = new Spark(0);
    robotMode = LEDState.OFF;

  }

  /** set by the buttons
   * - saves the game state
   * - sets the SD boolean box to color
   * - sets the physical LED
  */
  public void lightUp(LEDState ledState) {
   this.robotMode = ledState;

   blinkin.set(robotMode.colorValue);
  }

  public RobotMode getGameState() {
    return this.robotMode.mode;
  }

  public enum LEDState {
    OFF(0.99, RobotMode.NONE), 
    PICKUP(0.05, RobotMode.NOTE),
    SHOOT(0.69, RobotMode.SHOOT); 
    // CELEBRATION(0.05, GamePiece.NONE);

    public final double colorValue;
    public final RobotMode mode;

    LEDState(double colorValue, RobotMode robotMode)  {
        this.colorValue = colorValue;
        this.mode = robotMode;
    }
  }

  public enum RobotMode {
    NONE,
    NOTE,
    SHOOT,
    DRIVE
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Color", robotMode.toString());
  }
}
