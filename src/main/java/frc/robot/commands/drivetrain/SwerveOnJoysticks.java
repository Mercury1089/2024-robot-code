// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SwerveOnJoysticks extends Command {
  Drivetrain drivetrain;
  Supplier<Double> leftJoyX, leftJoyY, rightJoyX;
  double xSpeed, ySpeed, angularSpeed;
  /** Creates a new SwerveOnJoysticks. */
  public SwerveOnJoysticks(Drivetrain drivetrain, Supplier<Double> leftJoyX, Supplier<Double> leftJoyY, Supplier<Double> rightJoyX) {
    // Use addRequirements() here to declare subsystem dependencies.
    setName("SwerveOnJoysticks");
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.leftJoyX = leftJoyX;
    this.leftJoyY = leftJoyY;
    this.rightJoyX = rightJoyX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 0 if in deadband
    // (speed - deadband) / (1 - deadband)
    xSpeed = MathUtil.applyDeadband(leftJoyY.get(), SWERVE.JOYSTICK_DEADBAND);
    ySpeed = MathUtil.applyDeadband(leftJoyX.get(), SWERVE.JOYSTICK_DEADBAND);
    angularSpeed = MathUtil.applyDeadband(-rightJoyX.get(), SWERVE.JOYSTICK_DEADBAND);

    xSpeed = xSpeed > 0.0 ? Math.pow(xSpeed, 2) : -Math.pow(xSpeed, 2);
    ySpeed = ySpeed > 0.0 ? Math.pow(ySpeed, 2) : -Math.pow(ySpeed, 2);
    angularSpeed = angularSpeed > 0.0 ? Math.pow(angularSpeed, 2) : -Math.pow(angularSpeed, 2);
    
    
    SmartDashboard.putNumber("xSpeedJoystick", xSpeed);
    SmartDashboard.putNumber("ySpeedJoystick", ySpeed);
    SmartDashboard.putNumber("angularSpeedJoystick", angularSpeed);
    this.drivetrain.joyDrive(xSpeed, ySpeed, angularSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.drivetrain.joyDrive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
