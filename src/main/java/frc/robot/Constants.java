// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;

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
    //Add pipeline names later when all cameras are set up; grab from 10.3.42.11:5800
    public static final String LEFT_TURRET_CAMERA = "Photon_Vision_Turret_Left";
    public static final String RIGHT_TURRET_CAMERA = "Photon_Vision_Turret_Right";
    public static final String LEFT_ROBOT_CAMERA = "";
    public static final String RIGHT_ROBOT_CAMERA = "";

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    //Change these values after testing
    public static final double AMBIGUITY_CUTOFF = 0.1;
    public static final double TAG_CUTOFF_DISTANCE = 5.0;

    //Get all these values after cameras are put on robot
    public static final Translation3d LEFT_TURRET_CAMERA_TRANSLATION_3D = new Translation3d(0, 0, 0);
    public static final Translation3d RIGHT_TURRET_CAMERA_TRANSLATION_3D = new Translation3d(0, 0, 0);
    public static final Translation3d LEFT_ROBOT_CAMERA_TRANSLATION_3D = new Translation3d(0, 0, 0);
    public static final Translation3d RIGHT_ROBOT_CAMERA_TRANSLATION_3D = new Translation3d(0, 0, 0);

    public static final Rotation3d LEFT_TURRET_CAMERA_ROTATION_3D = new Rotation3d(0, 0, 0);
    public static final Rotation3d RIGHT_TURRET_CAMERA_ROTATION_3D = new Rotation3d(0, 0, 0);
    public static final Rotation3d LEFT_ROBOT_CAMERA_ROTATION_3D = new Rotation3d(0, 0, 0);
    public static final Rotation3d RIGHT_ROBOT_CAMERA_ROTATION_3D = new Rotation3d(0, 0, 0);

    public static final Transform3d LEFT_TURRET_CAMERA_TRANSFORM_3D = new Transform3d(LEFT_TURRET_CAMERA_TRANSLATION_3D, LEFT_TURRET_CAMERA_ROTATION_3D);
    public static final Transform3d RIGHT_TURRET_CAMERA_TRANSFORM_3D = new Transform3d(RIGHT_TURRET_CAMERA_TRANSLATION_3D, RIGHT_TURRET_CAMERA_ROTATION_3D);
    public static final Transform3d LEFT_ROBOT_CAMERA_TRANSFORM_3D = new Transform3d(LEFT_ROBOT_CAMERA_TRANSLATION_3D, LEFT_ROBOT_CAMERA_ROTATION_3D);
    public static final Transform3d RIGHT_ROBOT_CAMERA_TRANSFORM_3D = new Transform3d(RIGHT_ROBOT_CAMERA_TRANSLATION_3D, RIGHT_ROBOT_CAMERA_ROTATION_3D);
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

    //CANcoder IDs
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
    public static final double[] DRIVE_SVA_VALUES = {0.0952, 0.044888, 0.00558155};
    public static final double[] ROTATE_PID_VALUES = {0.3, 0.0, 0.4};

    //Max drive and rotate speeds
    public static final double MAX_DRIVE_SPEED = Units.feetToMeters(15);
    public static final double MAX_ROTATE_SPEED = 3 * Math.PI;

    //Min drive and rotate speeds
    public static final double MIN_DRIVE_SPEED = Units.feetToMeters(3);
    public static final double MIN_ROTATE_SPPEED = Math.PI;

    public static final PPHolonomicDriveController PATH_CONFIG_CONTROLLER = new PPHolonomicDriveController(
      new PIDConstants(1, 0, 0.7),
      new PIDConstants(0.25, 0, 0.3)
    );
  }

  public static class TurretConstants{
    public static final int TURRET_ID = 0;
    public static final int[] TURRET_PID_VALUES = {0, 0, 0};

    //Will be used if turret doesn't use a slip ring
    public static final double TURRET_MIN_ANGLE = 0; //The minimum angle the turret can safely be at; change later
    public static final double TURRET_MAX_ANGLE = 0; //The maximum angle the turret can safely be at; change later

    public static final double TURRET_GEAR_RATIO = 0;
    public static final double TURRET_POSITION_CONVERSION = 360/TURRET_GEAR_RATIO;

    public static final double TURRET_OFFSET_X = Units.feetToMeters(0); //update with other values later; according to field coordinates
    public static final double TURRET_OFFSET_Y = Units.feetToMeters(0); //update with other values later; according to field coordingates
  }

  public static class IntakeConstants{
    public static final int INTAKE_ID = 0;
    public static final int WRIST_ID = 0;
    public static final int WRIST_ENCODER_ID = 0;
    
    public static final double WRIST_GEAR_RATIO = 3.0;
    public static final double WRIST_POSITION_CONVERSION_FACTOR = (2*Math.PI) / WRIST_GEAR_RATIO;

    public static final double WRIST_ENCODER_ZERO_OFFSET = 0.0;

    public static final double WRIST_ALLOWED_ERROR = 0.0;
    public static final double WRIST_DOWN_POSITION = 0.0;
    public static final double WRIST_UP_POSITION = 0.0;
    
    public static final double[] WRIST_PID_VALUES = {0.0, 0.0, 0.0};
  }

  public static class ShooterConstants{
    public static final int SPINDEXER_ID = 0;
    public static final int TOP_SHOOTER_MOTOR_ID = 0;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 0;
    public static final int FEEDER_MOTOR_ID = 0;

    public static final int SHOOTER_VELOCITY_ERROR = 0;

    public static final double[] TOP_SHOOTER_PID_VALUES = {0.0, 0.0, 0.0};
    public static final double[] BOTTOM_SHOOTER_PID_VALUES = {0.0, 0.0, 0.0};
  }
}
