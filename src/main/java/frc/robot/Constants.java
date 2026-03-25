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
    public static final String TURRET_CAMERA = "Turret";
    public static final String ROBOT_RIGHT_CAMERA = "Robot_Right";
    public static final String ROBOT_LEFT_CAMERA = "Robot_Left";
    public static final String ROBOT_BACK_CAMERA = "Robot_Back";

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    //Change these values after testing
    public static final double AMBIGUITY_CUTOFF = 0.1;
    public static final double TAG_CUTOFF_DISTANCE = 5.0;

    //Get all these values after cameras are put on robot
    public static final Translation3d TURRET_CAMERA_TRANSLATION_3D = new Translation3d(0, 0, 0); //zero, because it's not used for pose estimation (and it's changing constantly if turret is moving)
    public static final Translation3d ROBOT_RIGHT_CAMERA_TRANSLATION_3D = new Translation3d(0.314, -0.314, 0.629);
    public static final Translation3d ROBOT_LEFT_CAMERA_TRANSLATION_3D = new Translation3d(0.314, 0.314, 0.629);
    public static final Translation3d ROBOT_BACK_CAMERA_TRANSLATION_3D = new Translation3d(-0.296, -0.046, 0.492);

    public static final Rotation3d TURRET_CAMERA_ROTATION_3D = new Rotation3d(0, 0, 0); //zero, because it's not used for pose estimation (and it's changing constantly if turret is moving)
    public static final Rotation3d ROBOT_RIGHT_CAMERA_ROTATION_3D = new Rotation3d(Math.PI/2, -1.023, -0.785);
    public static final Rotation3d ROBOT_LEFT_CAMERA_ROTATION_3D = new Rotation3d(Math.PI/2, -1.023, -0.785);
    public static final Rotation3d ROBOT_BACK_CAMERA_ROTATION_3D = new Rotation3d(0, -0.261799, Math.PI);

    public static final Transform3d TURRET_CAMERA_TRANSFORM_3D = new Transform3d(TURRET_CAMERA_TRANSLATION_3D, TURRET_CAMERA_ROTATION_3D);
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
    public static final double[] DRIVE_SVA_VALUES = {0.0952, 0.044888, 0.00558155};
    public static final double[] ROTATE_PID_VALUES = {0.4, 0.0, 0.4};

    //Max drive and rotate speeds
    public static final double MAX_DRIVE_SPEED = Units.feetToMeters(20);
    public static final double MAX_ROTATE_SPEED = 3 * Math.PI;

    //Min drive and rotate speeds
    public static final double MIN_DRIVE_SPEED = Units.feetToMeters(6);
    public static final double MIN_ROTATE_SPPEED = Math.PI;

    public static final PPHolonomicDriveController PATH_CONFIG_CONTROLLER = new PPHolonomicDriveController(
      new PIDConstants(1, 0, 0.7),
      new PIDConstants(1, 0, 1)
    );

    //NavX angle adjustment (degrees)
    public static final double NAVX_OFFSET = 0;
  }

  public static class TurretConstants{
    public static final int TURRET_ID = 17;
    public static final double[] TURRET_PID_VALUES_SLOT0 = {0.005, 0, 0};
    public static final double[] TURRET_PID_VALUES_SLOT1 = {0.04, 0, 0.015};

    public static final double TURRET_MIN_ANGLE = -90;
    public static final double TURRET_MAX_ANGLE = 160.0;
    public static final double TURRET_ALLOWED_ERROR = 0.5;

    public static final double TURRET_GEAR_RATIO = 12.5;
    public static final double TURRET_POSITION_CONVERSION = (2*Math.PI/TURRET_GEAR_RATIO) * 180/Math.PI;

    public static final double TURRET_OFFSET_X = -0.131;
    public static final double TURRET_OFFSET_Y = 0.151;

    public static final double THROUGHBORE_ZERO = 0.2887135572178389;
  }

  public static class IntakeConstants{
    public static final int INTAKE_ID = 13;
    public static final int WRIST_ID = 14;
    public static final int WRIST_ENCODER_ID = 0;
    
    public static final double WRIST_GEAR_RATIO = 25.0;
    public static final double WRIST_POSITION_CONVERSION_FACTOR = (2*Math.PI) / WRIST_GEAR_RATIO;

    public static final double WRIST_ALLOWED_ERROR = 0.025;
    public static final double WRIST_DOWN_POSITION = 5.861;
    public static final double WRIST_UP_POSITION = 0.01;
    public static final double WRIST_MIDDLE_POSITION = 2.182;
    
    public static final double[] WRIST_PID_VALUES_SLOT0 = {0.15, 0.0, 0.03};
    public static final double[] WRIST_PID_VALUES_SLOT1 = {0.2, 0.0, 0.25};
  }

  public static class ShooterConstants{
    public static final int SPINDEXER_ID = 15;
    public static final int TOP_SHOOTER_MOTOR_ID = 19;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 18;
    public static final int BOTTOM_FEEDER_MOTOR_ID = 16;
    public static final int TOP_FEEDER_MOTOR_ID = 20;

    public static final double SHOOTER_VELOCITY_ERROR = 0.1;

    public static final double[] TOP_SHOOTER_PID_VALUES = {0, 0, 0};
    public static final double[] TOP_SHOOTER_SVA_VALUES = {0, 0.476584, 0};

    public static final double[] BOTTOM_SHOOTER_PID_VALUES = {0, 0, 0};
    public static final double[] BOTTOM_SHOOTER_SVA_VALUES = {0, 0.349366, 0};

    public static final double BOTTOM_SHOOTER_WHEEL_DIAMETERS = 4.0;
    public static final double TOP_SHOOTER_WHEEL_DIAMETERS = 3.0;

    public static final double BOTTOM_SHOOTER_VELOCITY_CONVERSION_FACTOR = ((BOTTOM_SHOOTER_WHEEL_DIAMETERS * 0.0254) * Math.PI) / 60;
    public static final double TOP_SHOOTER_VELOCITY_CONVERSION_FACTOR = ((TOP_SHOOTER_WHEEL_DIAMETERS * 0.0254) * Math.PI) / 60;
  }
}
