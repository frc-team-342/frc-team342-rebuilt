// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
    //Add pipeline names later when all cameras are set up; grab from 10.3.42.11:5800
    // public static final String ROBOT_BL_CAMERA = "Robot_BL";
    public static final String ROBOT_RIGHT_CAMERA = "Robot_Right";
    public static final String ROBOT_LEFT_CAMERA = "Robot_Left";
    public static final String ROBOT_BACK_CAMERA = "Robot_Back";

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    //Change these values after testing
    public static final double AMBIGUITY_CUTOFF = 0.1;
    public static final double TAG_CUTOFF_DISTANCE = 5.0;

    //Get all these values after cameras are put on robot
    // public static final Translation3d ROBOT_BL_CAMERA_TRANSLATION_3D = new Translation3d(-0.78761, 0.324973, 0.403574);
    public static final Translation3d ROBOT_RIGHT_CAMERA_TRANSLATION_3D = new Translation3d(0.320152, -0.320152, 0.53697);
    public static final Translation3d ROBOT_LEFT_CAMERA_TRANSLATION_3D = new Translation3d(0.319615, 0.3195, 0.538693);
    public static final Translation3d ROBOT_BACK_CAMERA_TRANSLATION_3D = new Translation3d(-0.283451, 0.033655, 0.415173);

    // public static final Rotation3d ROBOT_BL_CAMERA_ROTATION_3D = new Rotation3d(Math.PI/2, Units.degreesToRadians(15), Units.degreesToRadians(120));
    public static final Rotation3d ROBOT_RIGHT_CAMERA_ROTATION_3D = new Rotation3d(Math.PI/2, Units.degreesToRadians(25.996633), Units.degreesToRadians(-50.538352));
    public static final Rotation3d ROBOT_LEFT_CAMERA_ROTATION_3D = new Rotation3d(-Math.PI/2, Units.degreesToRadians(25.996633), Units.degreesToRadians(50.538352));
    public static final Rotation3d ROBOT_BACK_CAMERA_ROTATION_3D = new Rotation3d(0, Units.degreesToRadians(15), Math.PI); //-0.261799

    // public static final Transform3d ROBOT_BL_CAMERA_TRANSFORM_3D = new Transform3d(ROBOT_BL_CAMERA_TRANSLATION_3D, ROBOT_BL_CAMERA_ROTATION_3D);
    public static final Transform3d ROBOT_RIGHT_CAMERA_TRANSFORM_3D = new Transform3d(ROBOT_RIGHT_CAMERA_TRANSLATION_3D, ROBOT_RIGHT_CAMERA_ROTATION_3D);
    public static final Transform3d ROBOT_LEFT_CAMERA_TRANSFORM_3D = new Transform3d(ROBOT_LEFT_CAMERA_TRANSLATION_3D, ROBOT_LEFT_CAMERA_ROTATION_3D);
    public static final Transform3d RIGHT_BACK_CAMERA_TRANSFORM_3D = new Transform3d(ROBOT_BACK_CAMERA_TRANSLATION_3D, ROBOT_BACK_CAMERA_ROTATION_3D);
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
    public static final double[] DRIVE_SVA_VALUES = {0.0952, 0.044888, 0.00558155}; //0.0952, 0.044888, 0.00558155
    public static final double[] ROTATE_PID_VALUES = {0.4, 0.0, 0.4};

    //Max drive and rotate speeds
    public static final double MAX_DRIVE_SPEED = Units.feetToMeters(20);
    public static final double MAX_ROTATE_SPEED = 3 * Math.PI;

    //Min drive and rotate speeds
    public static final double MIN_DRIVE_SPEED = Units.feetToMeters(6);
    public static final double MIN_ROTATE_SPPEED = Math.PI;

    public static final PPHolonomicDriveController PATH_CONFIG_CONTROLLER = new PPHolonomicDriveController(
      new PIDConstants(1, 0, 0.7),
      new PIDConstants(1.2, 0, 0.55)
    );

    public static final double FL_OFFSET = 0.0;
    public static final double FR_OFFSET = 0.0;
    public static final double BL_OFFSET = 0.0;
    public static final double BR_OFFSET = 0.0;

    //NavX angle adjustment (degrees)
    public static final double NAVX_OFFSET = 0;
  }

  public static class TurretConstants{
    public static final int TURRET_ID = 17;
    public static final double[] TURRET_PID_VALUES_SLOT0 = {0.005, 0, 0};
    public static final double[] TURRET_PID_VALUES_SLOT1 = {0.04, 0, 0.01};

    public static final double TURRET_MIN_ANGLE = -90;
    public static final double TURRET_MAX_ANGLE = 160.0;
    public static final double TURRET_ALLOWED_ERROR = 1;

    public static final double TURRET_GEAR_RATIO = 12.5;
    public static final double TURRET_POSITION_CONVERSION = (2*Math.PI/TURRET_GEAR_RATIO) * 180/Math.PI;

    public static final double TURRET_OFFSET_X = -0.131;
    public static final double TURRET_OFFSET_Y = 0.151;

    public static final Transform2d TURRET_OFFSET = new Transform2d(
      TURRET_OFFSET_X, TURRET_OFFSET_Y, new Rotation2d(180));

    public static final double THROUGHBORE_ZERO = 0.2887135572178389;
  }

  public static class IntakeConstants{
    public static final int INTAKE_ID = 13;
    public static final int WRIST_ID = 14;
    
    public static final double WRIST_GEAR_RATIO = 25.0;
    public static final double WRIST_POSITION_CONVERSION_FACTOR = (2*Math.PI) / WRIST_GEAR_RATIO;

    public static final double WRIST_ALLOWED_ERROR = 0.025;
    public static final double WRIST_DOWN_POSITION = 5.7065;
    public static final double WRIST_UP_POSITION = 0.01;
    public static final double WRIST_MIDDLE_POSITION = 2.182;
    
    public static final double[] WRIST_PID_VALUES_SLOT0 = {0.165, 0.0, 0.0295};
    public static final double[] WRIST_PID_VALUES_SLOT1 = {0.2, 0.0, 0.25};
  }

  public static class ShooterConstants{
    public static final int SPINDEXER_ID = 15;
    public static final int TOP_SHOOTER_MOTOR_ID = 19;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 18;
    public static final int BOTTOM_FEEDER_MOTOR_ID = 16;
    public static final int TOP_FEEDER_MOTOR_ID = 20;

    public static final double SHOOTER_VELOCITY_ERROR = 0.1;

    public static final double[] TOP_SHOOTER_PID_VALUES = {0.005, 0, 0};
    public static final double[] TOP_SHOOTER_SVA_VALUES = {0.16444, 0.4433449982132, 0}; //0.476584

    public static final double[] BOTTOM_SHOOTER_PID_VALUES = {0.01, 0, 0};
    public static final double[] BOTTOM_SHOOTER_SVA_VALUES = {0.14449, 0.3225087486599, 0}; //0.349366

    public static final double BOTTOM_SHOOTER_WHEEL_DIAMETERS = 4.0;
    public static final double TOP_SHOOTER_WHEEL_DIAMETERS = 3.0;

    public static final double BOTTOM_SHOOTER_VELOCITY_CONVERSION_FACTOR = ((BOTTOM_SHOOTER_WHEEL_DIAMETERS * 0.0254) * Math.PI) / 60;
    public static final double TOP_SHOOTER_VELOCITY_CONVERSION_FACTOR = ((TOP_SHOOTER_WHEEL_DIAMETERS * 0.0254) * Math.PI) / 60;
  }
}
