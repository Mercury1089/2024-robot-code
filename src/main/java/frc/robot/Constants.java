// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DS_USB {
    public static final int LEFT_STICK = 0, RIGHT_STICK = 1, GAMEPAD = 2;

    private DS_USB() {
    }
  }
  
  public static class JOYSTICK_BUTTONS {
    public static final int BTN1 = 1;
    public static final int BTN2 = 2;
    public static final int BTN3 = 3;
    public static final int BTN4 = 4;
    public static final int BTN5 = 5;
    public static final int BTN6 = 6;
    public static final int BTN7 = 7;
    public static final int BTN8 = 8;
    public static final int BTN9 = 9;
    public static final int BTN10 = 10;
    public static final int BTN11 = 11;

    public JOYSTICK_BUTTONS() {
    }
  }

  public static class APRILTAGS {

    // in relation to standing at (0,0) and facing the red speaker
    public static final int LEFT_BLUE_SOURCE = 1; 
    public static final int RIGHT_BLUE_SOURCE = 2;
    public static final int RIGHT_RED_SPEAKER = 3;
    public static final int MIDDLE_RED_SPEAKER = 4;
    public static final int RED_AMP = 5;
    public static final int BLUE_AMP = 6;
    public static final int MIDDLE_BLUE_SPEAKER = 7;
    public static final int RIGHT_BLUE_SPEAKER = 8;
    public static final int LEFT_BLUE_SOURCET_RED_SOURCE = 9;
    public static final int RIGHT_RED_SOURCE = 10;
    public static final int RIGHT_RED_TRAP = 11;
    public static final int LEFT_RED_TRAP = 12;
    public static final int CENTER_RED_TRAP = 13;
    public static final int CENTER_BLUE_TRAP = 14;
    public static final int LEFT_BLUE_TRAP = 15;
    public static final int RIGHT_BLUE_TRAP = 16;
    


    public APRILTAGS() {
    }
  }

  public static class SWERVE {
    // 2024 robot
    public static final double WHEEL_WIDTH = 23.5; // distance between front/back wheels (in inches)
    public static final double WHEEL_LENGTH = 28.5; // distance between left/right wheels (in inches)

    // // bolt
    // public static final double WHEEL_WIDTH = 27; // distance between front/back wheels (in inches)
    // public static final double WHEEL_LENGTH = 27; // distance between left/right wheels (in inches)

    // Distance from center to farthest wheel
    public static final double WHEEL_RADIUS = (new Translation2d(Units.inchesToMeters(WHEEL_WIDTH / 2),Units.inchesToMeters(WHEEL_WIDTH / 2))).getNorm();


    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
    public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)

    public static final double WHEEL_DIAMETER = 0.0762; // meters
    // changes drive speed (more pinions = zoom robot)
    public static final int PINION_TEETH = 13;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double MOTOR_REDUCTION = (45.0 * 22) / (PINION_TEETH * 15);

    // conversion factor to get swerve to return rotations --> meters
    // wheel circumference (2*pi*r) / MOTOR_REDUCTION
    public static final double METERS_CONVERSION = (WHEEL_DIAMETER * Math.PI)
        / MOTOR_REDUCTION; // meters
    public static final double VELOCITY_CONVERSION = METERS_CONVERSION / 60.0; // meters per second

    public static final double RADIANS_CONVERSION = 2 * Math.PI;
    public static final double RADIANS_VELOCITY_CONVERSION = RADIANS_CONVERSION / 60;

    public static final double DRIVING_MOTOR_FREE_SPEED = NEO_MOTOR_CONSTANTS.FREE_SPEED_RPMS / 60; // rps
    public static final double DRIVE_WHEEL_FREE_SPEED = 
    (DRIVING_MOTOR_FREE_SPEED * (WHEEL_DIAMETER * Math.PI)) / MOTOR_REDUCTION;

    public static final double MAX_DIRECTION_SPEED = 4; // meters per second
    public static final double MAX_ROTATIONAL_SPEED = 2 * Math.PI; // radians per second
    public static final double MAX_ACCELERATION = 3.0; //m/s^2

    public static final double JOYSTICK_DEADBAND = 0.15;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps 
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 20;// amps
  }

  public static final class NEO_MOTOR_CONSTANTS {
    public static final double FREE_SPEED_RPMS = 5676;
  }

  public static class UNITS {
    public static final double
      MAG_ENCODER_TICKS_PER_REVOLUTION = 4096,
      PIGEON_NATIVE_UNITS_PER_ROTATION = 8192,
      FALCON_ENCODER_TICKS_PER_REVOLUTION = 2048,
      MAX_ROLL_DEGREES = 90.0,
      PIGEON_ROLL_UNITS = 2048;


    private UNITS() {
    }
  }

  public static class CAN {
    public static final int DRIVING_FRONT_LEFT = 3;
    public static final int TURNING_FRONT_LEFT = 4;
    public static final int DRIVING_FRONT_RIGHT = 7;
    public static final int TURNING_FRONT_RIGHT = 8; 
    public static final int DRIVING_BACK_LEFT = 1;
    public static final int TURNING_BACK_LEFT = 2;  
    public static final int DRIVING_BACK_RIGHT = 5;
    public static final int TURNING_BACK_RIGHT = 6;
    
    public static final int PIGEON_DRIVETRAIN = 9;
    // public static final int WRIST_SPARKMAX = 10;
    public static final int SHOOTER = 11;
    // public static final int SHOOTER_BACK_SPARKMAX = 12;
    public static final int ARM_LEFT = 13;
    public static final int ARM_RIGHT = 14;
    public static final int INTAKE= 15;
    
    
    private CAN() {} 
  }

  public static class BREAKBEAM {
    public static final int INTAKE_BREAKBEAM = 0;
    public static final int SHOOTER_BREAKBEAM = 1;

    private BREAKBEAM() {}
  }

  public static class CAN_STATUS_FREQ {
    public static final int
        XTRA_HIGH = 5, // 5ms Very high - use sparingly
        HIGH = 10, // 10ms - High for important sensors
        NORMAL = 20, // 20ms - Match robot loop freq
        LOW = 100, // 100ms - Less frequent
        XTRA_LOW = 255; // 255ms - Maximum status frame period
    private CAN_STATUS_FREQ() {
    }
  }

  public static class CTRE {
    public static final int PRIMARY_PID_LOOP = 0;
    public static final int AUX_PID_LOOP = 1;
    public static final int TIMEOUT_MS = 10;

    private CTRE() {
    }
  }
}
