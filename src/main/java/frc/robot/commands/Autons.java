package frc.robot.commands;

import java.nio.file.Path;
import java.sql.Driver;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SWERVE;
import frc.robot.sensors.ObjectDetectionCamera;
import frc.robot.subsystems.RobotModeLEDs;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Intake.IntakeSpeed;
import frc.robot.subsystems.arm.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.KnownLocations;
import frc.robot.util.MercMath;
import frc.robot.util.PathUtils;
import frc.robot.util.TargetUtils;

public class Autons {

    private SendableChooser<Pose2d> startingPoseChooser;
    private SendableChooser<AutonTypes> multiNoteChooser;
    private SendableChooser<AutonTypes> autonTypeChooser;
    private Pose2d startingPose;
    private AutonTypes autonType; // multiNoteType;
    private Command autonCommand;

    private Alliance alliance;

    private final double ROTATION_P = 3.0;
    private final double TRANSLATION_P = 5.0;
    private static final double MAX_NOTE_DISTANCE = 1.25;

    private final Command DO_NOTHING = new PrintCommand("Do Nothing Auton");
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private RobotModeLEDs LEDs;
    private Shooter shooter;
    private ChoreoControlFunction choreoController;

    public Autons(Drivetrain drivetrain, Intake intake, Shooter shooter, Arm arm, RobotModeLEDs leds) {

        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;
        this.LEDs = leds;

        KnownLocations knownLocations = KnownLocations.getKnownLocations();
        this.alliance = knownLocations.alliance;

        // Starting config for Auton Choosers
        this.startingPose = knownLocations.DO_NOTHING;
        this.autonType = AutonTypes.DO_NOT_MOVE;
        // this.multiNoteType = AutonTypes.DO_NOT_MOVE;

        setChoosers(knownLocations);

        HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live your Constants class
                new PIDConstants(TRANSLATION_P, 0.0, 0.0), // Translation PID constants
                new PIDConstants(ROTATION_P, 0.0, 0.0), // Rotation PID constants
                SWERVE.MAX_SPEED_METERS_PER_SECOND, // Max module speed, in m/s
                SWERVE.WHEEL_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
        );

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                () -> drivetrain.getPose(), // Robot pose supplier
                (pose) -> drivetrain.resetPose(pose), // Method to reset odometry (will be called if your auto has starting pose)
                () -> drivetrain.getFieldRelativSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (chassisSpeeds) -> drivetrain.drive(chassisSpeeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                pathFollowerConfig,
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                }, // Never flip a path - all paths use absolute coordinates
                drivetrain // Reference to this subsystem to set requirements
        );

        PPHolonomicDriveController
                .setRotationTargetOverride(() -> TargetUtils.getRotationTargetOverride(this, drivetrain, intake, arm));

        choreoController = Choreo.choreoSwerveController(
                new PIDController(TRANSLATION_P, 0.0, 0.0),
                new PIDController(TRANSLATION_P, 0.0, 0.0),
                new PIDController(ROTATION_P, 0.0, 0.0));
    }

    public Command getAutonCommand() {
        return this.autonCommand;
    }

    public AutonTypes getAutonType() {
        return this.autonType;
    }

    public Command buildAutonCommand(KnownLocations knownLocations) {
        // SET OUR INITIAL POSE
        drivetrain.resetPose(startingPose);

        if (startingPose == knownLocations.DO_NOTHING) {
            SmartDashboard.putBoolean("isDoNothing", true);
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj1");
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj2");
            return DO_NOTHING;
        }
        SequentialCommandGroup autonCommand = new SequentialCommandGroup();

        PathPlannerPath path;
        int pathIndex = 1;

        // First, always score the preloaded NOTE
        autonCommand.addCommands(
                new InstantCommand(() -> LEDs.enableAutoShoot(), LEDs),
                shootNote());

        String autonToRun = "leaveStartingZone";

        switch (autonType) {
            case DO_NOT_MOVE:
                break;
            case LEAVE_STARTING_ZONE:
                autonToRun = "leaveStartingZone";
                break;
            case SCORE_2ND_NOTE:
                break;
            case MULTI_NOTE_SCORE:
                if (startingPose == knownLocations.START_TOPMOST) {
                    autonToRun = "leaveStartingZone";
                } else if (startingPose == knownLocations.START_BOTTOMMOST) {
                    autonToRun = "leaveStartingZone";
                } else if (startingPose == knownLocations.START_MIDDLE) {
                    autonToRun = "wingNoteAutonMiddle";
                }
                break;
            case CENTER_LINE_NOTES:
                if (startingPose == knownLocations.START_TOPMOST) {
                    autonToRun = "centerNoteAutonTopMost";
                } else if (startingPose == knownLocations.START_BOTTOMMOST) {
                    autonToRun = "centerNoteAutonBottomMost";
                }
                break;
        }

        PathPlannerAuto finalizedAuton = new PathPlannerAuto(autonToRun);

        // if (KnownLocations.getKnownLocations().alliance == Alliance.Red) {
        //     List<PathPlannerPath> pathsInAuto = PathPlannerAuto.getPathGroupFromAutoFile(autonToRun);
        //     for (PathPlannerPath p : pathsInAuto) {
        //         PathPlannerPath redSidedPath = p.flipPath();
        //         autonCommand.addCommands(
        //             AutoBuilder.followPath(redSidedPath),
        //             shootNote()
        //         );
        //     }
        // } else {
            
        // }

        autonCommand.addCommands(finalizedAuton);

        // drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(finalizedAuton), "autoRoutine");

        return autonCommand;
    }

    public boolean isReadyToShoot() {
        return intake.hasNote() &&
                drivetrain.isPointedAtTarget() &&
                drivetrain.isNotMoving() &&
                shooter.isAtTargetVelocity() &&
                arm.isFinishedMovingSpeaker() &&
                drivetrain.inShootingRange() &&
                LEDs.isAutoShootEnabled();
    }

    public boolean noteInRange() {
        ObjectDetectionCamera objectDetectionCam = drivetrain.getObjCam();
        return (objectDetectionCam.getLatestResult().hasTargets() &&
                objectDetectionCam.getDistanceToTarget() < MAX_NOTE_DISTANCE)
                || (intake.hasNote());
    }

    public Command setUpToShoot() {
        return DriveCommands.prepareToShoot(MercMath.zeroSupplier, MercMath.zeroSupplier, shooter, arm, drivetrain);
    }

    public Command pickUpNote() {
        return new SequentialCommandGroup(
            new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake).until(() -> intake.hasNote()),
            new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake)
        );
    }

    public Command shootNote() {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        setUpToShoot(),
                        new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake)).until(() -> isReadyToShoot()),
                new RunCommand(() -> intake.setSpeed(IntakeSpeed.SHOOT), intake)
                        .until(() -> !shooter.hasNote() && !intake.hasNote()),
                new ParallelCommandGroup(
                    new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm),
                    new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake)
                ).until(() -> arm.isAtPosition(ArmPosition.HOME))
            ),
            new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake).until(() -> !intake.hasNote()), 
            () -> intake.hasNote()
        );
    }


    /**
     * Rebuilds the autonCommand when ONE of the following conditions changes:
     * - Alliance Color
     * - Starting Pose
     * - Auton Type
     * - Multi Note Type
     */

    public void updateDash() {

        boolean rebuildAutonCommand = false;

        KnownLocations knownLocations = KnownLocations.getKnownLocations();

        if (knownLocations.alliance != this.alliance) {
            this.alliance = knownLocations.alliance;
            SmartDashboard.putString("alliance color!", this.alliance.toString());
            setChoosers(knownLocations);
            rebuildAutonCommand = true;
        }

        Pose2d startingPose = startingPoseChooser.getSelected();
        AutonTypes autonType = autonTypeChooser.getSelected();
        // AutonTypes multiNoteType = multiNoteChooser.getSelected();

        if (startingPose != this.startingPose) {
            this.startingPose = startingPose;
            rebuildAutonCommand = true;
        }

        if (autonType != this.autonType) {
            this.autonType = autonType;
            rebuildAutonCommand = true;
        }

        // if (multiNoteType != this.multiNoteType) {
        // this.multiNoteType = multiNoteType;
        // rebuildAutonCommand = true;
        // }

        if (rebuildAutonCommand) {
            this.autonCommand = buildAutonCommand(knownLocations);
        }
    }

    public void setChoosers(KnownLocations knownLocations) {
        // select the MANUAL STARTING POSITION of the robot
        this.startingPoseChooser = new SendableChooser<Pose2d>();
        this.startingPoseChooser.setDefaultOption("DO NOTHING", knownLocations.DO_NOTHING);
        this.startingPoseChooser.addOption("START TOP MOST", knownLocations.START_TOPMOST);
        this.startingPoseChooser.addOption("START MIDDLE", knownLocations.START_MIDDLE);
        this.startingPoseChooser.addOption("START BOTTOM MOST", knownLocations.START_BOTTOMMOST);
        SmartDashboard.putData("Starting Pose", startingPoseChooser);

        // select whether to visit charging station or score 2nd piece (or leave
        // community)
        this.autonTypeChooser = new SendableChooser<AutonTypes>();
        autonTypeChooser.setDefaultOption("LEAVE STARTING ZONE", AutonTypes.LEAVE_STARTING_ZONE);
        autonTypeChooser.addOption("2ND NOTE SCORE", AutonTypes.SCORE_2ND_NOTE);
        autonTypeChooser.addOption("MULTI NOTE SCORE", AutonTypes.MULTI_NOTE_SCORE);
        autonTypeChooser.addOption("CENTER LINE NOTE AUTO", AutonTypes.CENTER_LINE_NOTES);
        SmartDashboard.putData("Auton Type", autonTypeChooser);

        // select the ELEMENT to visit during auton (or DO NOTHING)
        // multiNoteChooser = new SendableChooser<AutonTypes>();
        // multiNoteChooser.setDefaultOption("DO NOTHING", AutonTypes.MULTI_NOTE_SCORE);
        // multiNoteChooser.addOption("WING NOTES", AutonTypes.WING_NOTES);
        // multiNoteChooser.addOption("CENTER LINE NOTES",
        // AutonTypes.CENTER_LINE_NOTES);
        // SmartDashboard.putData("Auton Element Chooser", multiNoteChooser);
    }

    /**
     * Determines what we after scoring initial note
     */
    public enum AutonTypes {
        DO_NOT_MOVE, // Do nothing after scoring first note
        LEAVE_STARTING_ZONE, // Leave the starting area after scoring first note
        SCORE_2ND_NOTE, // Score a second NOTE
        MULTI_NOTE_SCORE, // Score multiple NOTES
        WING_NOTES, // Score additional WING NOTES
        CENTER_LINE_NOTES // Score CENTER LINE NOTES
    }
}