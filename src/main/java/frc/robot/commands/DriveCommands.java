package frc.robot.commands;

import java.sql.Driver;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.APRILTAGS;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.FieldPosition;
import frc.robot.util.KnownLocations;
import frc.robot.util.MercMath;
import frc.robot.util.TargetUtils;

public class DriveCommands {

    public static Command joyStickDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> angularSpeedSupplier, Drivetrain drivetrain) {
        return new RunCommand(
            () -> drivetrain.drive(
              -MercMath.sqaureInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.sqaureInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.sqaureInput(MathUtil.applyDeadband(angularSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)))
          , drivetrain);
    }

    public static Command joyStickDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> angularSpeedSupplier, boolean fieldRelative, Drivetrain drivetrain) {
        return new RunCommand(
            () -> drivetrain.drive(
              -MercMath.sqaureInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.sqaureInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.sqaureInput(MathUtil.applyDeadband(angularSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              false)
          , drivetrain);
    } 

    public static Command targetDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, DoubleSupplier headingSupplier, Drivetrain drivetrain) {
        return new PIDCommand(
            drivetrain.getRotationalController(),
            () -> drivetrain.getPose().getRotation().getDegrees(),
            headingSupplier,
            (angularSpeed) -> drivetrain.drive(
              -MercMath.sqaureInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.sqaureInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
            angularSpeed),
            drivetrain);
    }

    public static Command targetDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, FieldPosition position, Drivetrain drivetrain) {
        return targetDrive(
            xSpeedSupplier, ySpeedSupplier,
            () -> TargetUtils.getTargetHeadingToFieldPosition(drivetrain.getPose(), position),
            drivetrain);
    }

    public static Command prepareToShoot(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Shooter shooter, Arm arm, Drivetrain drivetrain) {
        return new ParallelCommandGroup(
            DriveCommands.targetDrive(xSpeedSupplier, ySpeedSupplier, FieldPosition.SPEAKER, drivetrain),
            new RunCommand(() -> shooter.setVelocity(Shooter.SPEAKER_RPM), shooter),
            new RunCommand(() -> arm.setPosition(arm.getPosToTarget(arm.getDistanceToSpeaker())), arm));
    }

    public static Command ampTargetDrive(Supplier<Double> ySpeedSupplier, Drivetrain drivetrain) {
            return new RunCommand(() -> drivetrain.drive(
                drivetrain.getDirectionalController().calculate(drivetrain.getPose().getX(), KnownLocations.getFieldLayout().getTagPose(TargetUtils.getAmpTag()).get().getX()),
                (KnownLocations.getKnownLocations().alliance == Alliance.Blue ? -1 : 1) *
                MercMath.sqaureInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), -90.0),
                true,
                false,
                () -> drivetrain.getPose().getRotation()
            ), drivetrain);
    }
    
    public static Command shuttleNotesTargetDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Drivetrain drivetrain) {
        return DriveCommands.targetDrive(xSpeedSupplier, ySpeedSupplier, () -> TargetUtils.getTargetHeadingToPoint(drivetrain.getPose(), KnownLocations.getKnownLocations().WING_NOTE_MIDDLE.getTranslation()), drivetrain);
    }

    /**
     * Construct a command that will follow a path provided when the command initializes.
     * @param pathSupplier Supplier that provides the path to follow.
     * @param drivetrain Drivetrain subsystem that will follow the path
     * @return The 
     */
    public static Command followPath(Supplier<PathPlannerPath> pathSupplier, Drivetrain drivetrain) {
        return drivetrain.defer(() -> AutoBuilder.followPath(pathSupplier.get()));
    }


}
