package frc.robot.auton;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drivetrain.SwerveOnGyro;
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.LEDState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;


public class Autons {

    private SendableChooser<PathPoint> autonChooser;
    private SendableChooser<PathPoint> startingPoseChooser;
    private SendableChooser<AutonTypes> autonTypeChooser;
    private PathPoint currentSelectedAuton;
    private PathPoint currentSelectedPose;
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
    public Autons() {

    }

    public void setChoosers() {

        // select the ELEMENT to visit during auton (or DO NOTHING)
        autonChooser = new SendableChooser<PathPoint>();
        autonChooser.setDefaultOption("DO NOTHING", KnownLocations.DO_NOTHING);
        autonChooser.addOption("Element 1", knownLocations.ELEMENT1);
        autonChooser.addOption("Element 2", knownLocations.ELEMENT2);
        autonChooser.addOption("Element 3", knownLocations.ELEMENT3);
        autonChooser.addOption("Element 4", knownLocations.ELEMENT4);
        SmartDashboard.putData("Auton Element Chooser", autonChooser);
        SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());

        // select the MANUAL STARTING POSITION of the robot
        this.startingPoseChooser = new SendableChooser<PathPoint>();
        this.startingPoseChooser.setDefaultOption("TOPMOST", knownLocations.START_TOPMOST);
        this.startingPoseChooser.addOption("TOP SECOND", knownLocations.START_TOP_SECOND);
        this.startingPoseChooser.addOption("BOTTOM SECOND", knownLocations.START_BOTTOM_SECOND);
        this.startingPoseChooser.addOption("BOTTOMMOST", knownLocations.START_BOTTOMMOST);
        this.startingPoseChooser.addOption("MIDDLE CONE", knownLocations.START_MIDDLE_CONE);
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
        return null;
    }

    public PathPlannerTrajectory generateSwerveTrajectory(PathPoint initialPose, List<PathPoint> waypoints, PathPoint finalPose) {
        List<PathPoint> points = new ArrayList<PathPoint>();
        points.add(initialPose);
        points.addAll(waypoints);
        points.add(finalPose);
        // Following passes an array to vararg (see: https://programming.guide/java/passing-list-to-vararg-method.html)
        return PathPlanner.generatePath(pathConstraints, points);
        // return PathPlanner.generatePath(pathConstraints, initialPose, point2, points.toArray(new PathPoint[points.size()]));
    }

    /** Generate the swerve-specfic command by building the desired trajectory */
    public Command generateSwerveCommand(PathPlannerTrajectory trajectory) {
        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
            trajectory,
            () -> drivetrain.getPose(), // Functional interface to feed supplier
            drivetrain.getKinematics(),
            // Position controllers
            xController,
            yController,
            turningPIDController,
            (x) -> drivetrain.setModuleStates(x),
            false,
            drivetrain);
        return swerveControllerCommand;
    }

    /**
     * Rebuilds the autonCommand when ONE of the following conditions changes:
     * - Starting Pose
     * - Alliance Color
     * - Desired Element
     * - Auton Type
     */
    public void updateDash() {
        /*
        // runs constantly when disabled
        PathPoint currAuton = autonChooser.getSelected();
        PathPoint currPose = startingPoseChooser.getSelected();
        AutonTypes currAutonType = autonTypeChooser.getSelected();

        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        Alliance color = allianceColor.isPresent() ? allianceColor.get() : Alliance.Blue;

        if (color != this.allianceColor) {
            this.allianceColor = color;
            this.knownLocations = new KnownLocations();
            SmartDashboard.putString("alliance color!", this.allianceColor.toString());
            setChoosers();
            //this.autonCommand = buildAutonCommand();
        }
        
        if (currAuton != this.currentSelectedAuton) {
            this.currentSelectedAuton = currAuton;
            SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());
           // this.autonCommand = buildAutonCommand();
        }

        if (currPose != this.currentSelectedPose) {
            this.currentSelectedPose = currPose;
            //this.autonCommand = buildAutonCommand();
        }

        if (currAutonType != this.currentSelectedAutonType) {
            this.currentSelectedAutonType = currAutonType;
            //this.autonCommand = buildAutonCommand();
        }
        */
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