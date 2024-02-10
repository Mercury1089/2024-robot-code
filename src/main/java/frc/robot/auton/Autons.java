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
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
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
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.LEDState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.PathUtils;


public class Autons {

    private SendableChooser<Pose2d> firstElementChooser;
    private SendableChooser<Pose2d> startingPoseChooser;
    private SendableChooser<AutonTypes> autonTypeChooser;
    private Pose2d firstElement;
    private Pose2d currentSelectedPose;
    private Pose2d thirdPathPose;
    private Pose2d secondPathPose;
    private AutonTypes firstElementType;
    private PathConstraints pathConstraints;
    private PIDController turningPIDController;
    private PIDController xController, yController;
    private Command autonCommand;
    private KnownLocations knownLocations;
    private final int wingNote1 = 1;
    private final int wingNote2 = 2;
    private final int wingNote3 = 3;

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
        this.firstElement = knownLocations.WING_NOTE_1;
        this.currentSelectedPose = knownLocations.START_LEFT_NOTE;
        this.firstElementType = AutonTypes.LEAVE_STARTING_ZONE;

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
        firstElementChooser = new SendableChooser<Pose2d>();
        firstElementChooser.setDefaultOption("DO NOTHING", KnownLocations.DO_NOTHING);
        firstElementChooser.addOption("NOTE 1", knownLocations.WING_NOTE_1);
        firstElementChooser.addOption("NOTE 2", knownLocations.WING_NOTE_2);
        firstElementChooser.addOption("NOTE 3", knownLocations.WING_NOTE_3);
        SmartDashboard.putData("Auton Element Chooser", firstElementChooser);
        SmartDashboard.putString("Auton Selected: ", this.firstElement.toString());

        // secondElement = new SendableChooser<Pose2d>();
        // secondElement.setDefaultOption("DO NOTHING", KnownLocations.DO_NOTHING);
        // secondElement.addOption("NOTE 1", knownLocations.WING_NOTE_1);
        // secondElement.addOption("NOTE 2", knownLocations.WING_NOTE_2);
        // secondElement.addOption("NOTE 3", knownLocations.WING_NOTE_3);
        // secondElement.putData("Auton Element Chooser", firstElementChooser);
        // secondElement.putString("Auton Selected: ", this.secondElement.toString());

        // select the MANUAL STARTING POSITION of the robot
        this.startingPoseChooser = new SendableChooser<Pose2d>();
        this.startingPoseChooser.setDefaultOption("DO NOTHING", KnownLocations.DO_NOTHING);
        this.startingPoseChooser.addOption("START LEFT NOTE", knownLocations.START_LEFT_NOTE);
        this.startingPoseChooser.addOption("START MIDDLE NOTE", knownLocations.START_MID_NOTE);
        this.startingPoseChooser.addOption("START RIGHT NOTE", knownLocations.START_RIGHT_NOTE);
        SmartDashboard.putData("Manual Starting Pose", startingPoseChooser);

        // select whether to visit charging station or score 2nd piece (or leave community)
        this.autonTypeChooser = new SendableChooser<AutonTypes>();
        autonTypeChooser.setDefaultOption("LEAVE STARTING ZONE", AutonTypes.LEAVE_STARTING_ZONE);
        autonTypeChooser.addOption("2ND PIECE SCORE", AutonTypes.SCORE_2ND_PIECE);
        SmartDashboard.putData("Auton Type", autonTypeChooser);
    }
    
    public Command getAutonCommand() {
        return buildAutonCommand();
    }

    public Command buildAutonCommand() {        
        // SET OUR INITIAL POST
        drivetrain.setManualPose(currentSelectedPose);
        
        if (firstElement == KnownLocations.DO_NOTHING) {
            SmartDashboard.putBoolean("isDoNothing", true);
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj1");
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj2");
            return new PrintCommand("Do Nothing Auton");
        }
        

        Pose2d finalPose = knownLocations.WING_NOTE_1;
        List<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(knownLocations.INTERMEDIARY_NOTE_1);

        // if (firstElement == knownLocations.WING_NOTE_1) {
        //     waypoints = new ArrayList<Pose2d>();
        //     waypoints.add(knownLocations.WING_NOTE_1);
        // } else if (firstElement == knownLocations.WING_NOTE_2) {
        //     waypoints = new ArrayList<Pose2d>();
        //     waypoints.add(knownLocations.WING_NOTE_2);
        // } else if (firstElement == knownLocations.WING_NOTE_3) {
        //     waypoints = new ArrayList<Pose2d>();
        //     waypoints.add(knownLocations.WING_NOTE_3);
        // }
        PathPlannerPath path1, path2, path3;
        if (allianceColor == Alliance.Blue) {
            if (currentSelectedPose == knownLocations.START_LEFT_NOTE) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_1);
                finalPose = knownLocations.WING_NOTE_1;
            } else if (currentSelectedPose == knownLocations.START_MID_NOTE) {
                if (firstElement == knownLocations.WING_NOTE_1) {
                    waypoints = new ArrayList<Pose2d>();
                    waypoints.add(knownLocations.INTERMEDIARY_NOTE_1);
                    finalPose = knownLocations.WING_NOTE_1;
                } else if (firstElement == knownLocations.WING_NOTE_3) {
                    waypoints = new ArrayList<Pose2d>();
                    waypoints.add(knownLocations.INTERMEDIARY_NOTE_3);
                    finalPose = knownLocations.WING_NOTE_3;
                }
            } else if (currentSelectedPose == knownLocations.START_RIGHT_NOTE) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_3);
                finalPose = knownLocations.WING_NOTE_3;
            }

            path1 = generateSwerveTrajectory(currentSelectedPose, waypoints, finalPose);

            secondPathPose = finalPose == knownLocations.WING_NOTE_1 ? knownLocations.WING_NOTE_1 : knownLocations.WING_NOTE_3;
            waypoints = new ArrayList<Pose2d>();
            waypoints.add(knownLocations.INTERMEDIARY_NOTE_2);
            finalPose = knownLocations.WING_NOTE_2;

            path2 = generateSwerveTrajectory(secondPathPose, waypoints, finalPose);

            thirdPathPose = finalPose;
            waypoints = new ArrayList<Pose2d>();

            if (currentSelectedPose == knownLocations.START_LEFT_NOTE) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_3);
                finalPose = knownLocations.WING_NOTE_3;
            } else if (currentSelectedPose == knownLocations.START_MID_NOTE) {
                if (firstElement == knownLocations.WING_NOTE_1) {
                    waypoints = new ArrayList<Pose2d>();
                    waypoints.add(knownLocations.INTERMEDIARY_NOTE_3);
                    finalPose = knownLocations.WING_NOTE_3;
                } else if (firstElement == knownLocations.WING_NOTE_3) {
                    waypoints = new ArrayList<Pose2d>();
                    waypoints.add(knownLocations.INTERMEDIARY_NOTE_1);
                    finalPose = knownLocations.WING_NOTE_1;
                }
            } else if (currentSelectedPose == knownLocations.START_RIGHT_NOTE) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_1);
                finalPose = knownLocations.WING_NOTE_1;
            }

            path3 = generateSwerveTrajectory(thirdPathPose, waypoints, finalPose);

        } else {
            if (currentSelectedPose == knownLocations.START_LEFT_NOTE) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_3);
                finalPose = knownLocations.WING_NOTE_3;
            } else if (currentSelectedPose == knownLocations.START_MID_NOTE) {
                if (firstElement == knownLocations.WING_NOTE_1) {
                    waypoints = new ArrayList<Pose2d>();
                    waypoints.add(knownLocations.INTERMEDIARY_NOTE_1);
                    finalPose = knownLocations.WING_NOTE_1;
                } else if (firstElement == knownLocations.WING_NOTE_3) {
                    waypoints = new ArrayList<Pose2d>();
                    waypoints.add(knownLocations.INTERMEDIARY_NOTE_3);
                    finalPose = knownLocations.WING_NOTE_3;
                }
            } else if (currentSelectedPose == knownLocations.START_RIGHT_NOTE) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_1);
                finalPose = knownLocations.WING_NOTE_1;
            }

            path1 = generateSwerveTrajectory(currentSelectedPose, waypoints, finalPose);

            secondPathPose = finalPose == knownLocations.WING_NOTE_1 ? knownLocations.WING_NOTE_1 : knownLocations.WING_NOTE_3;
            waypoints = new ArrayList<Pose2d>();
            waypoints.add(knownLocations.INTERMEDIARY_NOTE_2);
            finalPose = knownLocations.WING_NOTE_2;

            path2 = generateSwerveTrajectory(secondPathPose, waypoints, finalPose);

            thirdPathPose = finalPose;
            waypoints = new ArrayList<Pose2d>();

            if (currentSelectedPose == knownLocations.START_LEFT_NOTE) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_1);
                finalPose = knownLocations.WING_NOTE_1;
            } else if (currentSelectedPose == knownLocations.START_MID_NOTE) {
                if (firstElement == knownLocations.WING_NOTE_1) {
                    waypoints = new ArrayList<Pose2d>();
                    waypoints.add(knownLocations.INTERMEDIARY_NOTE_3);
                    finalPose = knownLocations.WING_NOTE_3;
                } else if (firstElement == knownLocations.WING_NOTE_3) {
                    waypoints = new ArrayList<Pose2d>();
                    waypoints.add(knownLocations.INTERMEDIARY_NOTE_1);
                    finalPose = knownLocations.WING_NOTE_1;
                }
            } else if (currentSelectedPose == knownLocations.START_RIGHT_NOTE) {
                waypoints = new ArrayList<Pose2d>();
                waypoints.add(knownLocations.INTERMEDIARY_NOTE_3);
                finalPose = knownLocations.WING_NOTE_3;
            }

            path3 = generateSwerveTrajectory(thirdPathPose, waypoints, finalPose);
        }
        
        drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path1.getTrajectory(new ChassisSpeeds(), currentSelectedPose.getRotation())), "traj1");
        drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path2.getTrajectory(new ChassisSpeeds(), currentSelectedPose.getRotation())), "traj2");
        drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path3.getTrajectory(new ChassisSpeeds(), currentSelectedPose.getRotation())), "traj3");
        Command firstSwerveCommand = AutoBuilder.followPath(path1);
        Command secondSwerveCommand = AutoBuilder.followPath(path2);
        Command thirdSwerveCommand = AutoBuilder.followPath(path3);

        return new SequentialCommandGroup(
            firstSwerveCommand,
            secondSwerveCommand,
            thirdSwerveCommand
        );
    }

    public PathPlannerPath generateSwerveTrajectory(Pose2d initialPose, List<Pose2d> waypoints, Pose2d finalPose) {
        List<Pose2d> poses = new ArrayList<Pose2d>();
        poses.add(initialPose); poses.addAll(waypoints); poses.add(finalPose);

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);
        return new PathPlannerPath(
            bezierPoints,
            pathConstraints,
            new GoalEndState(0.0, finalPose.getRotation()));
    }

    public List<Pose2d> getWayPoints(int waypoint) {
        List<Pose2d> points = new ArrayList<Pose2d>();

        switch (waypoint) {
            case wingNote1:
                points.add(knownLocations.INTERMEDIARY_NOTE_1);
                points.add(knownLocations.WING_NOTE_1);
                return points;
            case wingNote2:
                points.add(knownLocations.INTERMEDIARY_NOTE_2);
                points.add(knownLocations.WING_NOTE_2);
                return points; 
            case wingNote3:
                points.add(knownLocations.INTERMEDIARY_NOTE_3);
                points.add(knownLocations.WING_NOTE_3);
                return points;
        }

        return points;
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
        Pose2d currAuton = firstElementChooser.getSelected();
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
        
        if (currAuton != this.firstElement) {
            this.firstElement = currAuton;
            SmartDashboard.putString("Auton Selected: ", this.firstElement.toString());
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