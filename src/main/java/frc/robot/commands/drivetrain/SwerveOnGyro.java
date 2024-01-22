// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveOnGyro extends PIDCommand {

  double THRESHOLD_DEGREES = 2.0;
  /** Creates a new SwerveOnGyro. */
  public SwerveOnGyro(Drivetrain drivetrain, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(0.12/14, 0, 0),
        // This should return the measurement
        () -> drivetrain.getRoll(),
        // This should return the setpoint (can also be a constant)
        setpoint,
        // This uses the output
        (output) -> drivetrain.joyDrive(output, 0.0, 0.0, false),
        drivetrain
    );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(THRESHOLD_DEGREES);
    getController().enableContinuousInput(-90.0, 90.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return getController().atSetpoint();
    return false;
  }

  /** must be negated */
  public boolean getTrue() {
    return false;
  }
}
