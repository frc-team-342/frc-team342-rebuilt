// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class DriveConstants {
    //Drive motor IDs
    public static final int FRONT_LEFT_DRIVE_ID = 0;
    public static final int FRONT_RIGHT_DRIVE_ID = 0;
    public static final int BACK_LEFT_DRIVE_ID = 0;
    public static final int BACK_RIGHT_DRIVE_ID = 0;
    
    //Rotate motor IDs
    public static final int FRONT_LEFT_ROTATE_ID = 0;
    public static final int FRONT_RIGHT_ROTATE_ID = 0;
    public static final int BACK_LEFT_ROTATE_ID = 0;
    public static final int BACK_RIGHT_ROTATE_ID = 0;

    //CANcoder IDs
    public static final int FRONT_LEFT_CANCODER_ID = 0;
    public static final int FRONT_RIGHT_CANCODER_ID = 0;
    public static final int BACK_LEFT_CANCODER_ID = 0;
    public static final int BACK_RIGHT_CANCODER_ID = 0;

    //Module offsets
    public static final double FL_OFFSET = 0.0;
    public static final double FR_OFFSET = 0.0;
    public static final double BL_OFFSET = 0.0;
    public static final double BR_OFFSET = 0.0;

    //Wheel Diameters
    public static final double FL_DIAMETER = 0.0;
    public static final double FR_DIAMETER = 0.0;
    public static final double BL_DIAMETER = 0.0;
    public static final double BR_DIAMETER = 0.0;

    //Module gear Ratios
    public static final double DRIVE_GEAR_RATIO = 0.0;
    public static final double ROTATE_GEAR_RATIO = 0.0;
    
    //Conversion factors
    public static final double ROTATE_POSITION_CONVERSION = (2*Math.PI)/ROTATE_GEAR_RATIO;
    public static final double ROTATE_VELOCITY_CONVERSION = ROTATE_POSITION_CONVERSION/60.0;

    //PIDF values
    public static final double[] DRIVE_PIDF_VALUES = {0.0, 0.0, 0.0, 0.0};
    public static final double[] ROTATE_PID_VALUES = {0.0, 0.0, 0.0};

    //Max drive and rotate speeds
    public static final double MAX_DRIVE_SPEED = 0;
    public static final double MAX_ROTATE_SPEED = 0;

    //Min drive and rotate speeds
    public static final double MIN_DRIVE_SPEED = 0;
    public static final double MIN_ROTATE_SPPEED = 0;
  }

  public static class TurretConstants{
    public static final int TURRET_ID = 0;
    public static final int[] TURRET_PID_VALUES = {0, 0, 0};

    //Will be used if turret doesn't use a slip ring
    public static final double TURRET_MIN_ANGLE = 0; //The minimum angle the turret can safely be at
    public static final double TURRET_MAX_ANGLE = 0; //The maximum angle the turret can safely be at

    public static final double TURRET_GEAR_RATIO = 0;
    public static final double TURRET_POSITION_CONVERSION = 360/TURRET_GEAR_RATIO;
  }
}
