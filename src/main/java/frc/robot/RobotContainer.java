// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DS_USB;
import frc.robot.Constants.JOYSTICK_BUTTONS;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.Autons;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.RobotModeLEDs;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Shooter;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.Intake.IntakeSpeed;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.FieldPosition;
import frc.robot.util.KnownLocations;
import frc.robot.util.MercMath;
import frc.robot.util.TargetUtils;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private CommandJoystick rightJoystick, leftJoystick;
  private CommandXboxController gamepad;

  private Trigger left1, left2, left3, left4, left5, left6, left7, left8, left9, left10, left11;
  private Trigger right1, right2, right3, right4, right5, right6, right7, right8, right9, right10, right11;
  private Trigger gamepadA, gamepadB, gamepadX, gamepadY, gamepadRB, gamepadLB, gamepadL3, gamepadBack, 
  gamepadStart, gamepadLeftStickButton, gamepadRightStickButton, gamepadLT, gamepadRT, gamepadPOVDown, gamepadPOVUpLeft, 
  gamepadPOVUp, gamepadPOVUpRight, gamepadPOVLeft, gamepadPOVRight, gamepadPOVDownRight, gamepadPOVDownLeft;

  private GenericHID gamepadHID;
  private Supplier<Double> gamepadLeftX, gamepadLeftY, gamepadRightX, gamepadRightY, rightJoystickX, rightJoystickY, leftJoystickX, leftJoystickY;

  private Autons auton;
  private RobotModeLEDs LEDs;
  private Arm arm;
  private Intake intake;
  private Drivetrain drivetrain;
  private Shooter shooter;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // buttons & bindings
    leftJoystick = new CommandJoystick(DS_USB.LEFT_STICK);
    rightJoystick = new CommandJoystick(DS_USB.RIGHT_STICK);
    gamepad = new CommandXboxController(DS_USB.GAMEPAD);
    gamepadHID = new GenericHID(DS_USB.GAMEPAD);
    configureBindings();

    drivetrain = new Drivetrain();
    drivetrain.setDefaultCommand(DriveCommands.joyStickDrive(leftJoystickY, leftJoystickX, rightJoystickX, drivetrain));
    drivetrain.resetGyro();

    arm = new Arm(drivetrain);
    arm.setDefaultCommand(new RunCommand(() -> arm.setSpeed(gamepadLeftY), arm));
    // arm.setDefaultCommand(new RunCommand(() -> arm.changePos(), arm));

    intake = new Intake();
    intake.setDefaultCommand(new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake));
    
    LEDs = new RobotModeLEDs(intake);
    LEDs.disableAutoShoot();

    shooter = new Shooter();
    shooter.setDefaultCommand(new RunCommand(() -> shooter.setVelocity(Shooter.STEADY_RPM), shooter));
    // shooter.setDefaultCommand(new RunCommand(() -> shooter.stopShooter(), shooter));
    
    auton = new Autons(drivetrain, intake, shooter, arm, LEDs);

    Map<String, Command> commands = new HashMap<String, Command>();

    commands.put("pickUpNote", auton.pickUpNote());
    commands.put("shootNote", auton.shootNote());

    NamedCommands.registerCommands(commands);

    Trigger intakeHasNote = new Trigger(() -> intake.hasNote() && DriverStation.isTeleop());
    intakeHasNote.onTrue(new ParallelCommandGroup(
      new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake)
    ));

    Trigger noNotePresent = new Trigger(() -> !intake.hasNote() && !shooter.hasNote() && DriverStation.isTeleop());
    
    noNotePresent.onTrue(
      new ParallelCommandGroup(
        new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm),
        new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake),
        new RunCommand(() -> shooter.setVelocity(Shooter.STEADY_RPM), shooter)
      )
    );

    Trigger noteInRange = new Trigger(() -> drivetrain.getObjCam().getLatestResult().hasTargets() && DriverStation.isTeleop());
    
    right1.and(noteInRange).whileTrue(
      DriveCommands.targetDrive(
          leftJoystickY, leftJoystickX,
          () -> TargetUtils.getTargetHeadingToClosestNote(drivetrain.getObjCam(), drivetrain.getPose()).getDegrees(),
          drivetrain)
    );

    right1.and(noNotePresent).whileTrue(
      new ParallelCommandGroup(
        new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake),
        new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm)
      ) 
    );

    Trigger setUpToShoot = new Trigger(() -> drivetrain.inShootingRange() && intake.hasNote() && LEDs.isAutoShootEnabled() && DriverStation.isTeleop());

    setUpToShoot.whileTrue(
      DriveCommands.prepareToShoot(leftJoystickY, leftJoystickX, shooter, arm, drivetrain)
    );
    
    Trigger shootTrigger = new Trigger(
      () -> auton.isReadyToShoot() && 
      DriverStation.isTeleop());

    shootTrigger.onTrue(new SequentialCommandGroup(
      new RunCommand(() -> intake.setSpeed(IntakeSpeed.SHOOT), intake).until(() -> !shooter.hasNote() && !intake.hasNote()),
      new ParallelCommandGroup(
        new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm),
        new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake),
        new RunCommand(() -> shooter.setVelocity(Shooter.STEADY_RPM), shooter)
      )
    ));

    gamepadLB.onTrue(new InstantCommand(() -> LEDs.enableAutoShoot(), LEDs));
    gamepadRB.onTrue(new InstantCommand(() -> LEDs.disableAutoShoot(), LEDs));

    right11.onTrue(new InstantCommand(() -> drivetrain.drive(0.0, 0.0, 0.0), drivetrain));
    gamepadX.onTrue(new InstantCommand(() -> drivetrain.drive(0.0, 0.0, 0.0), drivetrain));
    
    left10.onTrue(new InstantCommand(() -> drivetrain.resetGyro(), drivetrain).ignoringDisable(true));
    left11.onTrue(new RunCommand(() -> drivetrain.lockSwerve(), drivetrain));

    // for endgame
    gamepadPOVUp.onTrue(
      new ParallelCommandGroup(
        new RunCommand(() -> arm.setPosition(ArmPosition.AMP), arm),
        new RunCommand(() -> shooter.stopShooter(), shooter)
      )
    );

    gamepadPOVDown.onTrue( new ParallelCommandGroup(
         new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm),
         new RunCommand(() -> shooter.stopShooter(), shooter)
      )
    );

    gamepadY.onTrue(
      new ParallelCommandGroup(
        new RunCommand(() -> arm.setPosition(ArmPosition.AMP), arm),
        new RunCommand(() -> shooter.stopShooter(), shooter)
      )
    );
    
    gamepadB.whileTrue(new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake));

    gamepadA.onTrue(new SequentialCommandGroup(
      new RunCommand(() -> shooter.setVelocity(Shooter.AMP_RPM), shooter).until(() -> shooter.isAtAmpVelocity()),
      new RunCommand(() -> intake.setSpeed(IntakeSpeed.AMP), intake)
    ));

    left1.and(intakeHasNote).whileTrue(
      new SequentialCommandGroup(
        DriveCommands.ampTargetDrive(leftJoystickX, drivetrain).until(() -> TargetUtils.ampShotCheck(drivetrain.getPose()) && !LEDs.isAutoShootEnabled()),
        new ParallelCommandGroup(
          DriveCommands.ampTargetDrive(leftJoystickX, drivetrain),
          new RunCommand(() -> shooter.setVelocity(Shooter.AMP_RPM), shooter),
          new RunCommand(() -> arm.setPosition(ArmPosition.AMP), arm)
        )
      )
    );

    BooleanSupplier shuttleCheck = () -> arm.isAtPosition(ArmPosition.SHUTTLE) && shooter.isAtTargetVelocity() && drivetrain.isPointedAtShuttleTarget();

    left3.and(intakeHasNote).whileTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new RunCommand(() -> shooter.setVelocity(Shooter.STEADY_RPM), shooter),
          new RunCommand(() -> arm.setPosition(ArmPosition.SHUTTLE), arm),
          DriveCommands.shuttleNotesTargetDrive(gamepadLeftY, gamepadLeftX, drivetrain)
        ).until(shuttleCheck),
        new RunCommand(() -> intake.setSpeed(IntakeSpeed.SHOOT), intake)
      )
    ).onFalse(
      new ParallelCommandGroup(
        new RunCommand(() -> shooter.setVelocity(Shooter.STEADY_RPM), shooter),
        new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm)
      )
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        left1 = leftJoystick.button(JOYSTICK_BUTTONS.BTN1);
        left2 = leftJoystick.button(JOYSTICK_BUTTONS.BTN2);
        left3 = leftJoystick.button(JOYSTICK_BUTTONS.BTN3);
        left4 = leftJoystick.button(JOYSTICK_BUTTONS.BTN4);
        left5 = leftJoystick.button(JOYSTICK_BUTTONS.BTN5);
        left6 = leftJoystick.button(JOYSTICK_BUTTONS.BTN6);
        left7 = leftJoystick.button(JOYSTICK_BUTTONS.BTN7);
        left8 = leftJoystick.button(JOYSTICK_BUTTONS.BTN8);
        left9 = leftJoystick.button(JOYSTICK_BUTTONS.BTN9);
        left10 = leftJoystick.button(JOYSTICK_BUTTONS.BTN10);
        left11 = leftJoystick.button(JOYSTICK_BUTTONS.BTN11);

        right1 = rightJoystick.button(JOYSTICK_BUTTONS.BTN1);
        right2 = rightJoystick.button(JOYSTICK_BUTTONS.BTN2);
        right3 = rightJoystick.button(JOYSTICK_BUTTONS.BTN3);
        right4 = rightJoystick.button(JOYSTICK_BUTTONS.BTN4);
        right5 = rightJoystick.button(JOYSTICK_BUTTONS.BTN5);
        right6 = rightJoystick.button(JOYSTICK_BUTTONS.BTN6);
        right7 = rightJoystick.button(JOYSTICK_BUTTONS.BTN7);
        right8 = rightJoystick.button(JOYSTICK_BUTTONS.BTN8);
        right9 = rightJoystick.button(JOYSTICK_BUTTONS.BTN9);
        right10 = rightJoystick.button(JOYSTICK_BUTTONS.BTN10);
        right11 = rightJoystick.button(JOYSTICK_BUTTONS.BTN11);

        gamepadA = gamepad.a();
        gamepadB = gamepad.b();
        gamepadX = gamepad.x();
        gamepadY = gamepad.y();
        gamepadRB = gamepad.rightBumper();
        gamepadLB = gamepad.leftBumper();
        gamepadBack = gamepad.back();
        gamepadStart = gamepad.start();
        gamepadLeftStickButton = gamepad.leftStick();
        gamepadRightStickButton = gamepad.rightStick();
        gamepadLT = gamepad.leftTrigger();
        gamepadRT = gamepad.rightTrigger();
        
        gamepadPOVDown = gamepad.povDown();
        gamepadPOVUpLeft = gamepad.povUpLeft();
        gamepadPOVUp = gamepad.povUp();
        gamepadPOVUpRight = gamepad.povUpRight();
        gamepadPOVLeft = gamepad.povLeft();
        gamepadPOVRight = gamepad.povRight();
        gamepadPOVDownRight = gamepad.povDownRight();
        gamepadPOVDownLeft = gamepad.povDownLeft();

        gamepadLeftX = () -> gamepad.getLeftX();
        gamepadRightX = () -> gamepad.getRightX();
        gamepadLeftY = () -> -gamepad.getLeftY();
        gamepadRightY = () -> -gamepad.getRightY();

        leftJoystickX = () -> leftJoystick.getX();
        leftJoystickY = () -> leftJoystick.getY();
        rightJoystickX = () -> rightJoystick.getX();
        rightJoystickY = () -> rightJoystick.getY();


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Autons getAutonomous() {
    // An example command will be run in autonomous
    return auton;
  }
}
