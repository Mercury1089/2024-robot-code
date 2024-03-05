package frc.robot.auton;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.signals.Led1OffColorValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.APRILTAGS;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.RobotModeLEDs;
// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.Intake.IntakeSpeed;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.FieldPosition;
import frc.robot.util.MercMath;
import frc.robot.util.PathUtils;
import frc.robot.util.TargetUtils;


public class Autons {

    private SendableChooser<Pose2d> firstElementChooser;
    private SendableChooser<Pose2d> startingPoseChooser;
    private SendableChooser<AutonTypes> autonTypeChooser;
    private Pose2d firstElement;
    private Pose2d currentSelectedPose;
    private Pose2d thirdPathPose;
    private Pose2d secondPathPose;
    private AutonTypes firstElementType;
    private static PathConstraints pathConstraints = new PathConstraints(
        SWERVE.MAX_SPEED_METERS_PER_SECOND,
        SWERVE.MAX_ACCELERATION,
        SWERVE.MAX_ROTATIONAL_SPEED,
        SWERVE.MAX_ANGULAR_SPEED
    );

    private Command autonCommand;
    private KnownLocations knownLocations;

    private Alliance allianceColor;

    private final double ROTATION_P = 5.0;
    private final double TRANSLATION_P = 5.0;

    private final Command DO_NOTHING = new PrintCommand("Do Nothing Auton");
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private RobotModeLEDs LEDs;
    private Shooter shooter;

    /**
     * made by rohan no thanks to owen :(
     */
    public Autons(Drivetrain drivetrain, Intake intake, Shooter shooter, Arm arm, RobotModeLEDs leds) {

        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;
        this.LEDs = leds;

        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        this.allianceColor = allianceColor.isPresent() ? allianceColor.get() : Alliance.Blue;

        this.knownLocations = new KnownLocations();

        this.firstElement = KnownLocations.DO_NOTHING;
        this.currentSelectedPose = KnownLocations.DO_NOTHING;
        setChoosers();
        this.firstElement = firstElementChooser.getSelected();
        this.currentSelectedPose = startingPoseChooser.getSelected();
        // this.firstElementType = autonTypeChooser.getSelected();
        this.autonCommand = DO_NOTHING;
        

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                () -> drivetrain.getPose(), // Robot pose supplier
                (pose) -> drivetrain.setManualPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
                () -> drivetrain.getFieldRelativSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (chassisSpeeds) -> drivetrain.driveFieldRelative(chassisSpeeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(TRANSLATION_P, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(ROTATION_P, 0.0, 0.0), // Rotation PID constants
                        SWERVE.MAX_SPEED_METERS_PER_SECOND, // Max module speed, in m/s
                        SWERVE.WHEEL_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
                ),
                () -> { return false; }, // Never flip a path - all paths use absolute coordinates
                drivetrain // Reference to this subsystem to set requirements
        );
    }

    public void setChoosers() {

        // select the ELEMENT to visit during auton (or DO NOTHING)
        firstElementChooser = new SendableChooser<Pose2d>();
        firstElementChooser.setDefaultOption("DO NOTHING", KnownLocations.DO_NOTHING);
        firstElementChooser.addOption("NOTE TOP", knownLocations.WING_NOTE_TOP);
        firstElementChooser.addOption("NOTE MIDDLE", knownLocations.WING_NOTE_MIDDLE);
        firstElementChooser.addOption("NOTE BOTTOM", knownLocations.WING_NOTE_BOTTOM);
        SmartDashboard.putData("Auton Element Chooser", firstElementChooser);
        SmartDashboard.putString("Auton Selected: ", this.firstElement.toString());

        // secondElement = new SendableChooser<Pose2d>();
        // secondElement.setDefaultOption("DO NOTHING", KnownLocations.DO_NOTHING);
        // secondElement.addOption("NOTE 1", knownLocations.WING_NOTE_TOP);
        // secondElement.addOption("NOTE 2", knownLocations.WING_NOTE_MIDDLE);
        // secondElement.addOption("NOTE 3", knownLocations.WING_NOTE_BOTTOM);
        // secondElement.putData("Auton Element Chooser", firstElementChooser);
        // secondElement.putString("Auton Selected: ", this.secondElement.toString());

        // select the MANUAL STARTING POSITION of the robot
        this.startingPoseChooser = new SendableChooser<Pose2d>();
        this.startingPoseChooser.setDefaultOption("DO NOTHING", KnownLocations.DO_NOTHING);
        this.startingPoseChooser.addOption("START TOP MOST", knownLocations.START_TOPMOST);
        this.startingPoseChooser.addOption("START MIDDLE", knownLocations.START_MIDDLE);
        this.startingPoseChooser.addOption("START BOTTOM MOST", knownLocations.START_BOTTOMMOST);
        SmartDashboard.putData("Manual Starting Pose", startingPoseChooser);

        // select whether to visit charging station or score 2nd piece (or leave community)
        this.autonTypeChooser = new SendableChooser<AutonTypes>();
        autonTypeChooser.setDefaultOption("LEAVE STARTING ZONE", AutonTypes.LEAVE_STARTING_ZONE);
        autonTypeChooser.addOption("2ND PIECE SCORE", AutonTypes.SCORE_2ND_PIECE);
        SmartDashboard.putData("Auton Type", autonTypeChooser);
    }
    
    public Command getAutonCommand() {
        return this.autonCommand;
    }

    public Command buildAutonCommand() {        
        // SET OUR INITIAL POST
        drivetrain.setManualPose(currentSelectedPose);
        
        if (firstElement == KnownLocations.DO_NOTHING) {
            SmartDashboard.putBoolean("isDoNothing", true);
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj1");
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj2");
            return DO_NOTHING;
        }
        

        Pose2d finalPose = knownLocations.WING_NOTE_TOP;
        List<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(knownLocations.INTERMEDIARY_NOTE_TOP);

        // if (firstElement == knownLocations.WING_NOTE_TOP) {
        //     waypoints = new ArrayList<Pose2d>();
        //     waypoints.add(knownLocations.WING_NOTE_TOP);
        // } else if (firstElement == knownLocations.WING_NOTE_MIDDLE) {
        //     waypoints = new ArrayList<Pose2d>();
        //     waypoints.add(knownLocations.WING_NOTE_MIDDLE);
        // } else if (firstElement == knownLocations.WING_NOTE_BOTTOM) {
        //     waypoints = new ArrayList<Pose2d>();
        //     waypoints.add(knownLocations.WING_NOTE_BOTTOM);
        // }

        PathPlannerPath path1, path2, path3;
        
        if (currentSelectedPose == knownLocations.START_TOPMOST) {
            waypoints = new ArrayList<Pose2d>();
            waypoints.add(knownLocations.INTERMEDIARY_NOTE_TOP);
            finalPose = knownLocations.WING_NOTE_TOP;
        } else if (currentSelectedPose == knownLocations.START_MIDDLE) {
            if (firstElement == knownLocations.WING_NOTE_TOP) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_TOP);
                finalPose = knownLocations.WING_NOTE_TOP;
            } else if (firstElement == knownLocations.WING_NOTE_BOTTOM) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_BOTTOM);
                finalPose = knownLocations.WING_NOTE_BOTTOM;
            }
        } else if (currentSelectedPose == knownLocations.START_BOTTOMMOST) {
            waypoints = new ArrayList<Pose2d>();
            waypoints.add(knownLocations.INTERMEDIARY_NOTE_BOTTOM);
            finalPose = knownLocations.WING_NOTE_BOTTOM;
        }

        // path1 = generateSwerveTrajectory(currentSelectedPose, waypoints, finalPose);
        path1 = generateSwerveTrajectory(currentSelectedPose, new ArrayList<>(), knownLocations.LEAVE);
        

        secondPathPose = finalPose == knownLocations.WING_NOTE_TOP ? knownLocations.WING_NOTE_TOP : knownLocations.WING_NOTE_BOTTOM;
        waypoints = new ArrayList<Pose2d>();
        waypoints.add(knownLocations.INTERMEDIARY_NOTE_MIDDLE);
        finalPose = knownLocations.WING_NOTE_MIDDLE;

        path2 = generateSwerveTrajectory(secondPathPose, waypoints, finalPose);

        thirdPathPose = finalPose;
        waypoints = new ArrayList<Pose2d>();

        if (currentSelectedPose == knownLocations.START_TOPMOST) {
            waypoints = new ArrayList<Pose2d>();
            waypoints.add(knownLocations.INTERMEDIARY_NOTE_BOTTOM);
            finalPose = knownLocations.WING_NOTE_BOTTOM;
        } else if (currentSelectedPose == knownLocations.START_MIDDLE) {
            if (firstElement == knownLocations.WING_NOTE_TOP) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_BOTTOM);
                finalPose = knownLocations.WING_NOTE_BOTTOM;
            } else if (firstElement == knownLocations.WING_NOTE_BOTTOM) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_TOP);
                finalPose = knownLocations.WING_NOTE_TOP;
            }
        } else if (currentSelectedPose == knownLocations.START_BOTTOMMOST) {
            waypoints = new ArrayList<Pose2d>();
            waypoints.add(knownLocations.INTERMEDIARY_NOTE_TOP);
            finalPose = knownLocations.WING_NOTE_TOP;
        }

        path3 = generateSwerveTrajectory(thirdPathPose, waypoints, finalPose);
        
        drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path1.getTrajectory(new ChassisSpeeds(), currentSelectedPose.getRotation())), "traj1");
        // drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path2.getTrajectory(new ChassisSpeeds(), currentSelectedPose.getRotation())), "traj2");
        // drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path3.getTrajectory(new ChassisSpeeds(), currentSelectedPose.getRotation())), "traj3");
        Command firstSwerveCommand = AutoBuilder.followPath(path1);
        // Command secondSwerveCommand = AutoBuilder.followPath(path2);
        // Command thirdSwerveCommand = AutoBuilder.followPath(path3);

        // return new SequentialCommandGroup(
        //     setUpToShoot().until(() -> !shooter.hasNote() && !intake.hasNote()),

        //     firstSwerveCommand.until(() -> drivetrain.getObjCam().getLatestResult().getTargets().size() == 1 && arm.isAtPosition(ArmPosition.HOME)),
        //     new ParallelCommandGroup(
        //         drivetrain.goToNote(),
        //         new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake)
        //     ).until(() -> intake.hasNote()),
        //     setUpToShoot().until(() -> !shooter.hasNote() && !intake.hasNote()),

        //     secondSwerveCommand.until(() -> drivetrain.getObjCam().getLatestResult().getTargets().size() == 1 && arm.isAtPosition(ArmPosition.HOME)),
        //     new ParallelCommandGroup(
        //         drivetrain.goToNote(),
        //         new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake)
        //     ).until(() -> intake.hasNote()),
        //     setUpToShoot().until(() -> !shooter.hasNote() && !intake.hasNote()),

        //     thirdSwerveCommand.until(() -> drivetrain.getObjCam().getLatestResult().getTargets().size() == 1 && arm.isAtPosition(ArmPosition.HOME)),
        //     new ParallelCommandGroup(
        //         drivetrain.goToNote(),
        //         new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake)
        //     ).until(() -> intake.hasNote()),
        //     setUpToShoot().until(() -> !shooter.hasNote() && !intake.hasNote())
        // );
        
        return new SequentialCommandGroup(
            setUpToShoot().until(() -> isReadyToShoot()),
            new RunCommand(() -> intake.setSpeed(IntakeSpeed.SHOOT), intake).until(() -> !shooter.hasNote() && !intake.hasNote()),
            new SequentialCommandGroup(
                new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm).until(() -> arm.isAtPosition(ArmPosition.HOME)),
                firstSwerveCommand // .until(() -> drivetrain.getObjCam().getLatestResult().getTargets().size() == 1 && arm.isAtPosition(ArmPosition.HOME)),
                // new ParallelCommandGroup(
                //     drivetrain.goToNote(),
                //     new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake)
                // ).until(() -> intake.hasNote())
            )
            //setUpToShoot().until(() -> isReadyToShoot()),
            // new RunCommand(() -> intake.setSpeed(IntakeSpeed.SHOOT), intake).until(() -> !shooter.hasNote() && !intake.hasNote())  
        );
    }

    public boolean isReadyToShoot() {
        return intake.hasNote() && 
        drivetrain.isPointedAtTarget() && 
        drivetrain.isNotMoving() &&
        shooter.isAtTargetVelocity() &&
        arm.isFinishedMoving() &&
        drivetrain.inShootingRange() &&
        LEDs.isAutoShootEnabled();
    }

    public Command setUpToShoot() {
        return new ParallelCommandGroup(
            new RunCommand(() -> LEDs.enableAutoShoot(), LEDs),
            new PIDCommand(
              drivetrain.getRotationalController(),
              () -> drivetrain.getPose().getRotation().getDegrees(), 
              () -> TargetUtils.getTargetHeadingToFieldPosition(drivetrain.getAprilTagCamera(), drivetrain.getPose(), FieldPosition.SPEAKER), 
              (angularSpeed) -> drivetrain.joyDrive(
                0.0,
                0.0,
              angularSpeed),
              drivetrain),
            new RunCommand(() -> shooter.setVelocity(Shooter.SPEAKER_RPM), shooter),
            new RunCommand(() -> arm.setPosition(arm.getPosToTarget(arm.getDistanceToSpeaker())), arm)
          );
    }

    public PIDCommand rotateToTarget() {
        return new PIDCommand(
            drivetrain.getRotationalController(),
            () -> drivetrain.getPose().getRotation().getDegrees(), 
            () -> TargetUtils.getTargetHeadingToFieldPosition(drivetrain.getAprilTagCamera(), drivetrain.getPose(), FieldPosition.SPEAKER), 
            (angularSpeed) -> drivetrain.joyDrive(
                -MercMath.sqaureInput(MathUtil.applyDeadband(0.0, SWERVE.JOYSTICK_DEADBAND)),
                -MercMath.sqaureInput(MathUtil.applyDeadband(0.0, SWERVE.JOYSTICK_DEADBAND)),
            angularSpeed),
            drivetrain);
    }

    public static PathPlannerPath generateSwerveTrajectory(Pose2d initialPose, List<Pose2d> waypoints, Pose2d finalPose, Rotation2d endStateRotation) {
        List<Pose2d> poses = new ArrayList<Pose2d>();
        poses.add(initialPose); poses.addAll(waypoints); poses.add(finalPose);

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);
        return new PathPlannerPath(
            bezierPoints,
            pathConstraints,
            new GoalEndState(0.0, endStateRotation));
    }

    public static PathPlannerPath generateSwerveTrajectory(Pose2d initialPose, List<Pose2d> waypoints, Pose2d finalPose) {
        return generateSwerveTrajectory(initialPose, waypoints, finalPose, finalPose.getRotation());
    }

    /**
     * Rebuilds the autonCommand when ONE of the following conditions changes:
     * - Starting Pose
     * - Alliance Color
     * - Desired Element
     * - Auton Type
     */

    public void updateDash() {
    
        boolean rebuildAutonCommand = false;
        
        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        Alliance color = allianceColor.isPresent() ? allianceColor.get() : Alliance.Blue;

        if (color != this.allianceColor) {
            this.allianceColor = color;
            this.knownLocations = new KnownLocations();
            SmartDashboard.putString("alliance color!", this.allianceColor.toString());
            setChoosers();
            rebuildAutonCommand = true;
        }

        // runs constantly when disabled
        Pose2d currAuton = firstElementChooser.getSelected();
        Pose2d currPose = startingPoseChooser.getSelected();
        AutonTypes currAutonType = autonTypeChooser.getSelected();
        
        if (currAuton != this.firstElement) {
            this.firstElement = currAuton;
            // SmartDashboard.putString("Auton Selected: ", this.firstElement.toString());
            rebuildAutonCommand = true;
        }

        if (currPose != this.currentSelectedPose) {
            this.currentSelectedPose = currPose;
            rebuildAutonCommand = true;
        }

        if (currAutonType != this.firstElementType) {
            this.firstElementType = currAutonType;
            rebuildAutonCommand = true;
        }

        if (rebuildAutonCommand) {
            this.autonCommand = buildAutonCommand();
        }
    }
    
    /**
     * Determines what we do with our 2nd path
     * 
     * LEAVE COMMUNITY - no 2nd path
     * SCORE_2ND_PIECE - return to appropriate node
     * CHARGING_STATION - center onto the charging station
     */
    public enum AutonTypes { 
        LEAVE_STARTING_ZONE,
        SCORE_2ND_PIECE
    }
}