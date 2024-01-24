// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.MercMath;

public class MoveArm extends Command {
  /** Creates a new MoveArm. */
  Arm arm;
  public final double THRESHOLD_DEGREES = 2.0;
  public MoveArm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  /*
  @Override
  public boolean isFinished() {
    return arm.getError() < MercMath.inchesToEncoderTicks(THRESHOLD_DEGREES);
  }
  */
}
