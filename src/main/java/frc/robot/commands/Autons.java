package frc.robot.commands;

import java.nio.file.Path;
import java.util.function.BooleanSupplier;

import javax.management.InstanceAlreadyExistsException;
import javax.swing.text.html.ParagraphView;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.RobotModeLEDs;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.Intake.IntakeSpeed;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.FieldPosition;
import frc.robot.util.KnownLocations;
import frc.robot.util.MercMath;
import frc.robot.util.PathUtils;
import frc.robot.util.TargetUtils;


public class Autons {

    private SendableChooser<Pose2d> startingPoseChooser;
    private SendableChooser<AutonTypes> multiNoteChooser;
    private SendableChooser<AutonTypes> autonTypeChooser;
    private Pose2d startingPose;
    private AutonTypes autonType, multiNoteType;
    private Command autonCommand;

    private Alliance alliance;

    private final double ROTATION_P = 3.0;
    private final double TRANSLATION_P = 5.0;

    private final Command DO_NOTHING = new PrintCommand("Do Nothing Auton");
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private RobotModeLEDs LEDs;
    private Shooter shooter;

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
        this.multiNoteType = AutonTypes.DO_NOT_MOVE;

        setChoosers(knownLocations);    
        
        HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(TRANSLATION_P, 0.0, 0.0), // Translation PID constants
            new PIDConstants(ROTATION_P, 0.0, 0.0), // Rotation PID constants
            SWERVE.MAX_SPEED_METERS_PER_SECOND, // Max module speed, in m/s
            SWERVE.WHEEL_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
        );

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                () -> drivetrain.getPose(), // Robot pose supplier
                (pose) -> drivetrain.setManualPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
                () -> drivetrain.getFieldRelativSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (chassisSpeeds) -> drivetrain.drive(chassisSpeeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                pathFollowerConfig,
                () -> { return false; }, // Never flip a path - all paths use absolute coordinates
                drivetrain // Reference to this subsystem to set requirements
        );

        PPHolonomicDriveController.setRotationTargetOverride(() -> TargetUtils.getRotationTargetOverride(drivetrain, intake, arm));
    }

    public void setChoosers(KnownLocations knownLocations) {
        // select the MANUAL STARTING POSITION of the robot
        this.startingPoseChooser = new SendableChooser<Pose2d>();
        this.startingPoseChooser.setDefaultOption("DO NOTHING", knownLocations.DO_NOTHING);
        this.startingPoseChooser.addOption("START TOP MOST", knownLocations.START_TOPMOST);
        this.startingPoseChooser.addOption("START MIDDLE", knownLocations.START_MIDDLE);
        this.startingPoseChooser.addOption("START BOTTOM MOST", knownLocations.START_BOTTOMMOST);
        SmartDashboard.putData("Starting Pose", startingPoseChooser);


        // select whether to visit charging station or score 2nd piece (or leave community)
        this.autonTypeChooser = new SendableChooser<AutonTypes>();
        autonTypeChooser.setDefaultOption("LEAVE STARTING ZONE", AutonTypes.LEAVE_STARTING_ZONE);
        autonTypeChooser.addOption("2ND NOTE SCORE", AutonTypes.SCORE_2ND_NOTE);
        autonTypeChooser.addOption("MULTI NOTE SCORE", AutonTypes.MULTI_NOTE_SCORE);
        autonTypeChooser.addOption("MULTI NOTE SCORE", AutonTypes.CENTER_LINE_NOTES);
        SmartDashboard.putData("Auton Type", autonTypeChooser);

        // select the ELEMENT to visit during auton (or DO NOTHING)
        multiNoteChooser = new SendableChooser<AutonTypes>();
        multiNoteChooser.setDefaultOption("DO NOTHING", AutonTypes.MULTI_NOTE_SCORE);
        multiNoteChooser.addOption("WING NOTES", AutonTypes.WING_NOTES);
        multiNoteChooser.addOption("CENTER LINE NOTES", AutonTypes.CENTER_LINE_NOTES);
        SmartDashboard.putData("Auton Element Chooser", multiNoteChooser);
    }
    
    public Command getAutonCommand() {
        return this.autonCommand;
    }

    public Command buildAutonCommand(KnownLocations knownLocations) {        
        // SET OUR INITIAL POSE
        drivetrain.setManualPose(startingPose);
        
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
            shootNote()
        );      

        switch(autonType) {
            case DO_NOT_MOVE:
                break;
            case LEAVE_STARTING_ZONE:
                path = PathUtils.generatePath(startingPose, knownLocations.LEAVE);
                drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path), "traj" + pathIndex);
                pathIndex++;
                autonCommand.addCommands(AutoBuilder.followPath(path));
                break;
            case SCORE_2ND_NOTE:
                if (startingPose == knownLocations.START_TOPMOST) {
                    path = PathUtils.generatePath(startingPose, knownLocations.WING_NOTE_TOP);
                } else if (startingPose == knownLocations.START_MIDDLE) {
                    path = PathUtils.generatePath(startingPose, knownLocations.WING_NOTE_MIDDLE);            
                } else {
                    path = PathUtils.generatePath(startingPose, knownLocations.WING_NOTE_BOTTOM);                 
                }

                drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path), "traj" + pathIndex);
                pathIndex++;
                autonCommand.addCommands(
                    pickUpNote(path),
                    shootNote()
                );
                break;
            case MULTI_NOTE_SCORE:
                path = PathUtils.generatePath(startingPose, knownLocations.INTERMEDIARY_NOTE_TOP, knownLocations.WING_NOTE_TOP);
                drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path), "traj" + pathIndex);
                pathIndex++;
                autonCommand.addCommands(
                    pickUpNote(path),
                    new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake).until(() -> intake.hasNote()),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(PathUtils.generatePath(drivetrain.getPose(), knownLocations.INTERMEDIARY_NOTE_TOP)),
                        new RunCommand(() -> arm.setPosition(arm.getPosToTarget(arm.getDistanceToSpeaker())), arm)
                    ).until(() -> drivetrain.isNotMoving() && arm.isFinishedMovingSpeaker()),
                    shootNote()
                );

                if (knownLocations.alliance == Alliance.Blue) {
                    path = PathUtils.generatePath(Rotation2d.fromDegrees(-60.0), startingPose, knownLocations.WING_NOTE_MIDDLE);
                } else {
                    path = PathUtils.generatePath(Rotation2d.fromDegrees(-120.0), startingPose, knownLocations.WING_NOTE_MIDDLE);
                }
                drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path), "traj" + pathIndex);
                pathIndex++;
                autonCommand.addCommands(
                    pickUpNote(path),
                    new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake).until(() -> intake.hasNote()),
                    shootNote()
                );

                if (knownLocations.alliance == Alliance.Blue) {
                    path = PathUtils.generatePath(Rotation2d.fromDegrees(-60.0), startingPose, knownLocations.INTERMEDIARY_NOTE_BOTTOM);
                } else {
                    path = PathUtils.generatePath(Rotation2d.fromDegrees(-120.0), startingPose, knownLocations.INTERMEDIARY_NOTE_BOTTOM);
                }
                drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path), "traj" + pathIndex);
                pathIndex++;
                autonCommand.addCommands(
                    pickUpNote(path),
                    new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake).until(() -> intake.hasNote()),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(PathUtils.generatePath(drivetrain.getPose(), knownLocations.INTERMEDIARY_NOTE_BOTTOM)),
                        new RunCommand(() -> arm.setPosition(arm.getPosToTarget(arm.getDistanceToSpeaker())), arm)
                    ).until(() -> drivetrain.isNotMoving() && arm.isFinishedMovingSpeaker()),
                    shootNote()
                );
                break;
            case CENTER_LINE_NOTES:
                path = PathUtils.generatePath(startingPose, knownLocations.CENTER_LINE_SHOOT, knownLocations.LEAVE, 
                    knownLocations.INTERMEDIARY_CENTER_NOTE_BOTTOM, knownLocations.INTERMEDIARY_CENTER_NOTE_SECOND_BOTTOM, knownLocations.INTERMEDIARY_CENTER_NOTE_MIDDLE, knownLocations.INTERMEDIARY_CENTER_NOTE_SECOND_TOP, knownLocations.INTERMEDIARY_CENTER_NOTE_TOP);
                drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path), "traj" + pathIndex);
                pathIndex++;
                
                autonCommand.addCommands(
                    pickUpCenterNote(path, knownLocations),
                    new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake).until(() -> intake.hasNote()),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(PathUtils.generatePath(drivetrain.getPose(), knownLocations.INTERMEDIARY_CENTER_NOTE_BOTTOM, knownLocations.LEAVE, knownLocations.CENTER_LINE_SHOOT)),
                        new RunCommand(() -> arm.setPosition(arm.getPosToTarget(arm.getDistanceToSpeaker())), arm)
                    ).until(() -> drivetrain.isNotMoving() && arm.isFinishedMovingSpeaker()),
                    shootNote(),

                    pickUpCenterNote(path, knownLocations),
                    new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake).until(() -> intake.hasNote()),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(PathUtils.generatePath(drivetrain.getPose(), knownLocations.INTERMEDIARY_CENTER_NOTE_BOTTOM, knownLocations.LEAVE, knownLocations.CENTER_LINE_SHOOT)),
                        new RunCommand(() -> arm.setPosition(arm.getPosToTarget(arm.getDistanceToSpeaker())), arm)
                    ).until(() -> drivetrain.isNotMoving() && arm.isFinishedMovingSpeaker()),
                    shootNote()
                );
        }
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

    public Command setUpToShoot() {
        return DriveCommands.prepareToShoot(MercMath.zeroSupplier, MercMath.zeroSupplier, shooter, arm, drivetrain);
    }

    public Command pickUpNote(PathPlannerPath path) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                // also can try making intake be on
                new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake),
                new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm),
                AutoBuilder.followPath(path)
            ).until(() -> (arm.isAtPosition(ArmPosition.HOME) && drivetrain.getObjCam().getLatestResult().getTargets().size() == 1)),
            new ParallelCommandGroup(
                drivetrain.defer(() -> AutoBuilder.followPath(drivetrain.getPathToNote())),
                new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake)
            ).until(() -> intake.hasNote())
        );
    }

    public Command pickUpCenterNote(PathPlannerPath path, KnownLocations knownLocations) {
        double passX = alliance == Alliance.Blue ?
            knownLocations.WING_NOTE_BOTTOM.getX() :
            knownLocations.WING_NOTE_BOTTOM.getX();

        BooleanSupplier pickUpNoteCheck = alliance == Alliance.Blue ?
            () -> arm.isAtPosition(ArmPosition.HOME) && drivetrain.getObjCam().getLatestResult().getTargets().size() == 1 && drivetrain.getPose().getY() > passX :
            () -> arm.isAtPosition(ArmPosition.HOME) && drivetrain.getObjCam().getLatestResult().getTargets().size() == 1 && drivetrain.getPose().getY() < passX;

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                // also can try making intake be on
                new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake),
                new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm),
                AutoBuilder.followPath(path)
            ).until(pickUpNoteCheck),
            new ParallelCommandGroup(
                drivetrain.defer(() -> AutoBuilder.followPath(drivetrain.getPathToNote())),
                new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake)
            ).until(() -> intake.hasNote())
        );
    }

    public Command shootNote() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                setUpToShoot(),
                new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake)
            ).until(() -> isReadyToShoot()),
            new RunCommand(() -> intake.setSpeed(IntakeSpeed.SHOOT), intake).until(() -> !shooter.hasNote() && !intake.hasNote())
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
        AutonTypes multiNoteType = multiNoteChooser.getSelected();

        if (startingPose != this.startingPose) {
            this.startingPose = startingPose;
            rebuildAutonCommand = true;
        }

        if (autonType != this.autonType) {
            this.autonType = autonType;
            rebuildAutonCommand = true;
        }

        if (multiNoteType != this.multiNoteType) {
            this.multiNoteType = multiNoteType;
            rebuildAutonCommand = true;
        }

        if (rebuildAutonCommand) {
            this.autonCommand = buildAutonCommand(knownLocations);
        }
    }
    
    /**
     * Determines what we after scoring initial note
     */
    public enum AutonTypes {
        DO_NOT_MOVE,         // Do nothing after scoring first note
        LEAVE_STARTING_ZONE,// Leave the starting area after scoring first note
        SCORE_2ND_NOTE,    // Score a second NOTE
        MULTI_NOTE_SCORE,    // Score multiple NOTES
        WING_NOTES,         // Score additional WING NOTES
        CENTER_LINE_NOTES   // Score CENTER LINE NOTES
    }
}