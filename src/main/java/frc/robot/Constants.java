// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class VisionConstants {
    /*
     * The 4 constants below are used to hold the names of each camera on the robot.
     * These names are set in the PhotonVision Client, at 10.3.42.12:5800 and 10.3.42.13:5800. 
     */
    // public static final String ROBOT_BL_CAMERA = "Robot_BL";
    public static final String ROBOT_RIGHT_CAMERA = "Robot_Right"; //The camera on the front right of the robot.
    public static final String ROBOT_LEFT_CAMERA = "Robot_Left"; //The camera on the front left of the robot.
    public static final String ROBOT_BACK_CAMERA = "Robot_Back"; //The camera on the back of the robot.

    /*
     * The FIELD_LAYOUT constant is used to hold a JSON file of the layout of april tags on the field.
     * Make sure that the correct version of the field is loaded (AndyMark vs Welded).
     * This is required to create an instance of the PhotonPoseEstimator class, which is used for pose estimation.
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /*
     * These 2 constants are used to increase the quality of tag readings used for pose estimation.
     * If the ambiguity of a pose is too high, the pose will be discarded.
     * If a tag is too far, the pose will be discarded.
     */
    public static final double AMBIGUITY_CUTOFF = 0.1;
    public static final double TAG_CUTOFF_DISTANCE = 5.0;

    /*
     * The 4 Translation3d constants are used for storing the X, Y, and Z offsets of the camera.
     * All these measurements are in meters and are relative to the center of the robot.
     * X is forward/backward (forward-positive), Y is left/right (left-positive), and Z is up/down (up-positive).
     */
    // public static final Translation3d ROBOT_BL_CAMERA_TRANSLATION_3D = new Translation3d(-0.78761, 0.324973, 0.403574);
    public static final Translation3d ROBOT_RIGHT_CAMERA_TRANSLATION_3D = new Translation3d(0.320152, -0.320152, 0.53697);
    public static final Translation3d ROBOT_LEFT_CAMERA_TRANSLATION_3D = new Translation3d(0.319615, 0.3195, 0.538693);
    public static final Translation3d ROBOT_BACK_CAMERA_TRANSLATION_3D = new Translation3d(-0.283451, 0.033655, 0.415173);

    /*
     * The 4 Rotation3d constants are used for storing the roll, pitch, and yaw offsets of the camera.
     * All these measurements are in radians and are relative to the starting rotation of the robot.
     * Roll is rotation around the X-axis, pitch is rotation around the Y-axis, and yaw is rotation around the Z-axis.
     * For all three, counterclockwise is positive.
     */
    // public static final Rotation3d ROBOT_BL_CAMERA_ROTATION_3D = new Rotation3d(Math.PI/2, Units.degreesToRadians(15), Units.degreesToRadians(120));
    public static final Rotation3d ROBOT_RIGHT_CAMERA_ROTATION_3D = new Rotation3d(Math.PI/2, Units.degreesToRadians(25.996633), Units.degreesToRadians(-50.538352));
    public static final Rotation3d ROBOT_LEFT_CAMERA_ROTATION_3D = new Rotation3d(-Math.PI/2, Units.degreesToRadians(25.996633), Units.degreesToRadians(50.538352));
    public static final Rotation3d ROBOT_BACK_CAMERA_ROTATION_3D = new Rotation3d(0, Units.degreesToRadians(15), Math.PI); //-0.261799

    /**
     * The 4 Transform3d constants are used for storing both the positional and rotational offsets of the camera.
     * An instance of Transform3d can be created using a Translation3d and Rotation3d.
     * In this case, the previous Translation3d and Rotation3d constants are used.
     */
    // public static final Transform3d ROBOT_BL_CAMERA_TRANSFORM_3D = new Transform3d(ROBOT_BL_CAMERA_TRANSLATION_3D, ROBOT_BL_CAMERA_ROTATION_3D);
    public static final Transform3d ROBOT_RIGHT_CAMERA_TRANSFORM_3D = new Transform3d(ROBOT_RIGHT_CAMERA_TRANSLATION_3D, ROBOT_RIGHT_CAMERA_ROTATION_3D);
    public static final Transform3d ROBOT_LEFT_CAMERA_TRANSFORM_3D = new Transform3d(ROBOT_LEFT_CAMERA_TRANSLATION_3D, ROBOT_LEFT_CAMERA_ROTATION_3D);
    public static final Transform3d ROBOT_BACK_CAMERA_TRANSFORM_3D = new Transform3d(ROBOT_BACK_CAMERA_TRANSLATION_3D, ROBOT_BACK_CAMERA_ROTATION_3D);
  }

  public static class DriveConstants {
    //Drive motor IDs
    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_ID = 2;
    public static final int BACK_LEFT_DRIVE_ID = 3;
    public static final int BACK_RIGHT_DRIVE_ID = 4;
    
    //Rotate motor IDs
    public static final int FRONT_LEFT_ROTATE_ID = 5;
    public static final int FRONT_RIGHT_ROTATE_ID = 6;
    public static final int BACK_LEFT_ROTATE_ID = 7;
    public static final int BACK_RIGHT_ROTATE_ID = 8;

    /*
     * CANcoder IDs
     * CANCoders are absolute encoders that use magnets to determine the starting position of
     * the encoders. For swerve drive, absolute encoders are typically used to ensure the
     * wheels and relative encoders all start at the same position.
     */
    public static final int FRONT_LEFT_CANCODER_ID = 9;
    public static final int FRONT_RIGHT_CANCODER_ID = 10;
    public static final int BACK_LEFT_CANCODER_ID = 11;
    public static final int BACK_RIGHT_CANCODER_ID = 12;

    //Wheel Diameters
    public static final double FL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double FR_DIAMETER = Units.inchesToMeters(4.0);
    public static final double BL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double BR_DIAMETER = Units.inchesToMeters(4.0);

    //Module gear Ratios
    public static final double DRIVE_GEAR_RATIO = 6.03;
    public static final double ROTATE_GEAR_RATIO = 26.0;
    
    //Conversion factors
    public static final double ROTATE_POSITION_CONVERSION = (2*Math.PI)/ROTATE_GEAR_RATIO;
    public static final double ROTATE_VELOCITY_CONVERSION = ROTATE_POSITION_CONVERSION/60.0;
    public static final double DRIVE_POSITION_CONVERSION = ((Math.PI * FL_DIAMETER) / DRIVE_GEAR_RATIO);
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION/60.0;

    //PIDF values
    public static final double[] DRIVE_PIDF_VALUES = {0.23, 0, 0.7, 0};
    public static final double[] DRIVE_SVA_VALUES = {0.0952, 0.044888, 0.00558155}; //0.0952, 0.044888, 0.00558155
    public static final double[] ROTATE_PID_VALUES = {0.4, 0.0, 0.4};

    //Max drive and rotate speeds
    public static final double MAX_DRIVE_SPEED = Units.feetToMeters(20);
    public static final double MAX_ROTATE_SPEED = 3 * Math.PI;

    //Min drive and rotate speeds
    public static final double MIN_DRIVE_SPEED = Units.feetToMeters(6);
    public static final double MIN_ROTATE_SPPEED = Math.PI;

    //PID values for PathPlanner
    public static final PPHolonomicDriveController PATH_CONFIG_CONTROLLER = new PPHolonomicDriveController(
      new PIDConstants(1, 0, 0.7),
      new PIDConstants(1.2, 0, 0.55)
    );

    /*
     * CANCoder offsets
     * Offsets are used to ensure that all 4 CANCoders (which are absolute encoders) have the same zero position.
     */
    public static final double FL_OFFSET = -0.022216796875;
    public static final double FR_OFFSET = -0.3671875;
    public static final double BL_OFFSET = -0.330810546875;
    public static final double BR_OFFSET = -0.251708984375;

    //NavX angle adjustment (degrees)
    public static final double NAVX_OFFSET = 0;
  }

  public static class TurretConstants{
    public static final int TURRET_ID = 17; //Turret motor ID

    /*
     * Turret PID values.
     * There are two different sets, because the spring mechanism for the turret
     * puts up different amounts of resistance depending on the direction the turret turns.
     * By using two different PID slots, the turret can turn smoothly in both directions without
     * struggling to turn in one direction or greatly overshooting in the other direction.
     */
    public static final double[] TURRET_PID_VALUES_SLOT0 = {0.005, 0, 0};
    public static final double[] TURRET_PID_VALUES_SLOT1 = {0.04, 0, 0.01};

    public static final double TURRET_MIN_ANGLE = -90; //The minimum angle the turret can turn to (counterclockwise)
    public static final double TURRET_MAX_ANGLE = 160.0; //The maximum angle the turret can turn to (clockwise)
    public static final double TURRET_ALLOWED_ERROR = 1.0; //The margin of error allowed for the turret (degrees)

    public static final double TURRET_GEAR_RATIO = 12.5; //Turret gear ratio
    public static final double TURRET_POSITION_CONVERSION = (2*Math.PI/TURRET_GEAR_RATIO) * 180/Math.PI; //Position conversion factor

    /*
     * Turret positional offsets. These are used to determine the position of the turret relative
     * to the center of the robot, since our turret is not centered.
     */
    public static final double TURRET_OFFSET_X = -0.131;
    public static final double TURRET_OFFSET_Y = 0.151;

    public static final Transform2d TURRET_OFFSET = new Transform2d(
      TURRET_OFFSET_X, TURRET_OFFSET_Y, new Rotation2d(180)
    );
  }

  public static class IntakeConstants{
    public static final int INTAKE_ID = 13; //Intake motor ID
    public static final int WRIST_ID = 14; //Wrist motor ID
    
    public static final double WRIST_GEAR_RATIO = 25.0; //Wrist gear ratio
    public static final double WRIST_POSITION_CONVERSION_FACTOR = (2*Math.PI) / WRIST_GEAR_RATIO; //Wrist position conversion factor

    public static final double WRIST_ALLOWED_ERROR = 0.025; //Amount of error allowed for the wrist PID
    /*
     * These 3 values are from the encoder reading when the wrist was manually set in each desired position.
     * The down position brings the wrist to the floor for intaking, the up position is the starting position,
     * and the middle position brings the wrist up to allow the robot to move around while holding fuel without
     * potentially damaging the wrist and intake mechanism.
     */
    public static final double WRIST_DOWN_POSITION = 5.7065;
    public static final double WRIST_UP_POSITION = 0.01;
    public static final double WRIST_MIDDLE_POSITION = 2.182;
    
    /*
     * Two slots are used for the wrist PID to ensure that it is able to move up/down smoothly.
     * Whlie moving up, it is fighting against gravity; the opposite is true when moving down.
     * This means that more force will have to be put in when moving the wrist up rather than down.
     */
    public static final double[] WRIST_PID_VALUES_SLOT0 = {0.165, 0.0, 0.0295};
    public static final double[] WRIST_PID_VALUES_SLOT1 = {0.2, 0.0, 0.25};
  }

  public static class ShooterConstants{
    public static final int SPINDEXER_ID = 15; //Spindexer motor ID
    public static final int TOP_SHOOTER_MOTOR_ID = 19; //Top shooter motor ID
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 18; //Bottom shooter motor ID
    public static final int BOTTOM_FEEDER_MOTOR_ID = 16; //Bottom feeder motor ID
    public static final int TOP_FEEDER_MOTOR_ID = 20; //Top feeder motor ID

    public static final double SHOOTER_VELOCITY_ERROR = 0.1; //The amount of allowed error in the velocity of the shooter (m/s)

    /*
     * The top and bottom shooter motors have separate PID and SVA values due to differing
     * physical characteristics (e.g., size). This means that some values may work for one but
     * not the other, so different values were obtained for each motor.
     */
    public static final double[] TOP_SHOOTER_PID_VALUES = {0.005, 0, 0};
    public static final double[] TOP_SHOOTER_SVA_VALUES = {0.16444, 0.4433449982132, 0}; //0.476584
    public static final double[] BOTTOM_SHOOTER_PID_VALUES = {0.01, 0, 0};
    public static final double[] BOTTOM_SHOOTER_SVA_VALUES = {0.14449, 0.3225087486599, 0}; //0.349366

    //Shooter wheel diameters
    public static final double BOTTOM_SHOOTER_WHEEL_DIAMETERS = 4.0;
    public static final double TOP_SHOOTER_WHEEL_DIAMETERS = 3.0;

    //Shooter velocity conversion factors
    public static final double BOTTOM_SHOOTER_VELOCITY_CONVERSION_FACTOR = ((BOTTOM_SHOOTER_WHEEL_DIAMETERS * 0.0254) * Math.PI) / 60;
    public static final double TOP_SHOOTER_VELOCITY_CONVERSION_FACTOR = ((TOP_SHOOTER_WHEEL_DIAMETERS * 0.0254) * Math.PI) / 60;
  }
}
