package frc.robot.auton;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.drivetrain.SwerveOnGyro;
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.LEDState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.PathUtils;


public class Autons {

    private SendableChooser<Pose2d> autonChooser;
    private SendableChooser<Pose2d> startingPoseChooser;
    private SendableChooser<AutonTypes> autonTypeChooser;
    private Pose2d currentSelectedAuton;
    private Pose2d currentSelectedPose;
    private AutonTypes currentSelectedAutonType;
    private PathConstraints pathConstraints;
    private PIDController turningPIDController;
    private PIDController xController, yController;
    private Command autonCommand;
    private KnownLocations knownLocations;

    private Alliance allianceColor;

    private final double TURNING_P_VAL = 1;
    private final double X_P_VAL = 1, Y_P_VAL = 1;
    private final double MAX_DIRECTIONAL_SPEED = 2, MAX_ACCELERATION = 2.0;

    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private GamePieceLEDs LEDs;

    /**
     * made by rohan no thanks to owen :(
     */
    public Autons(Drivetrain drivetrain) {

        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        this.allianceColor = allianceColor.isPresent() ? allianceColor.get() : Alliance.Blue;

        this.knownLocations = new KnownLocations();
        this.currentSelectedAuton = KnownLocations.DO_NOTHING;
        this.currentSelectedPose = KnownLocations.DO_NOTHING;
        this.currentSelectedAutonType = AutonTypes.LEAVE_COMMUNITY;

        this.drivetrain = drivetrain;
        this.pathConstraints = new PathConstraints(
            SWERVE.MAX_SPEED_METERS_PER_SECOND,
            SWERVE.MAX_ACCELERATION,
            SWERVE.MAX_ROTATIONAL_SPEED,
            SWERVE.MAX_ANGULAR_SPEED
        ); 
        turningPIDController = new PIDController(TURNING_P_VAL, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        xController = new PIDController(X_P_VAL, 0, 0);
        yController = new PIDController(Y_P_VAL, 0, 0);
        
        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                () -> drivetrain.getPose(), // Robot pose supplier
                (pose) -> drivetrain.setManualPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
                () -> drivetrain.getFieldRelativSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (chassisSpeeds) -> drivetrain.driveFieldRelative(chassisSpeeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> { return false; },
                drivetrain // Reference to this subsystem to set requirements
        );
        setChoosers();
    }

    public void setChoosers() {

        // select the ELEMENT to visit during auton (or DO NOTHING)
        autonChooser = new SendableChooser<Pose2d>();
        autonChooser.setDefaultOption("DO NOTHING", KnownLocations.DO_NOTHING);
        // autonChooser.addOption("Element 1", knownLocations.ELEMENT1);
        // autonChooser.addOption("Element 2", knownLocations.ELEMENT2);
        // autonChooser.addOption("Element 3", knownLocations.ELEMENT3);
        // autonChooser.addOption("Element 4", knownLocations.ELEMENT4);
        SmartDashboard.putData("Auton Element Chooser", autonChooser);
        SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());

        // select the MANUAL STARTING POSITION of the robot
        this.startingPoseChooser = new SendableChooser<Pose2d>();
        this.startingPoseChooser.setDefaultOption("TOPMOST", KnownLocations.DO_NOTHING);
        // this.startingPoseChooser.addOption("TOP SECOND", knownLocations.START_TOP_SECOND);
        // this.startingPoseChooser.addOption("BOTTOM SECOND", knownLocations.START_BOTTOM_SECOND);
        // this.startingPoseChooser.addOption("BOTTOMMOST", knownLocations.START_BOTTOMMOST);
        // this.startingPoseChooser.addOption("MIDDLE CONE", knownLocations.START_MIDDLE_CONE);
        SmartDashboard.putData("Manual Starting Pose", startingPoseChooser);

        // select whether to visit charging station or score 2nd piece (or leave community)
        this.autonTypeChooser = new SendableChooser<AutonTypes>();
        autonTypeChooser.setDefaultOption("LEAVE COMMUNITY", AutonTypes.LEAVE_COMMUNITY);
        autonTypeChooser.addOption("2ND PIECE SCORE", AutonTypes.SCORE_2ND_PIECE);
        autonTypeChooser.addOption("CHARGING STATION", AutonTypes.CHARGING_STATION);
        SmartDashboard.putData("Auton Type", autonTypeChooser);
    }
    
    public Command getAutonCommand() {
        return buildAutonCommand();
    }

    public Command buildAutonCommand() {        
        // SET OUR INITIAL POST
        drivetrain.setManualPose(currentSelectedPose);
        
        if (currentSelectedAuton == KnownLocations.DO_NOTHING) {
            SmartDashboard.putBoolean("isDoNothing", true);
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj1");
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj2");
            return new PrintCommand("Do Nothing Auton");
        }

        Pose2d finalPose = currentSelectedPose;
        List<Pose2d> waypoints = List.of();
        PathPlannerPath  path1 = generateSwerveTrajectory(currentSelectedAuton, waypoints, finalPose);

        drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path1.getTrajectory(new ChassisSpeeds(), currentSelectedPose.getRotation())), "traj1");
        Command firstSwerveCommand = AutoBuilder.followPath(path1);

        return firstSwerveCommand;
    }

    public PathPlannerPath generateSwerveTrajectory(Pose2d initialPose, List<Pose2d> waypoints, Pose2d finalPose) {
        List<Pose2d> poses = List.of();
        poses.add(initialPose); poses.addAll(waypoints); poses.add(finalPose);
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);
        return new PathPlannerPath(
            bezierPoints,
            pathConstraints,
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));
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
        
        // runs constantly when disabled
        Pose2d currAuton = autonChooser.getSelected();
        Pose2d currPose = startingPoseChooser.getSelected();
        AutonTypes currAutonType = autonTypeChooser.getSelected();

        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        Alliance color = allianceColor.isPresent() ? allianceColor.get() : Alliance.Blue;

        if (color != this.allianceColor) {
            this.allianceColor = color;
            this.knownLocations = new KnownLocations();
            SmartDashboard.putString("alliance color!", this.allianceColor.toString());
            setChoosers();
            rebuildAutonCommand = true;
        }
        
        if (currAuton != this.currentSelectedAuton) {
            this.currentSelectedAuton = currAuton;
            SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());
            rebuildAutonCommand = true;
        }

        if (currPose != this.currentSelectedPose) {
            this.currentSelectedPose = currPose;
            rebuildAutonCommand = true;
        }

        if (currAutonType != this.currentSelectedAutonType) {
            this.currentSelectedAutonType = currAutonType;
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
        LEAVE_COMMUNITY,
        SCORE_2ND_PIECE,
        CHARGING_STATION
    }
}