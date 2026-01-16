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
    public static final int FRONT_LEFT_DRIVE_ID = 0;
    public static final int FRONT_RIGHT_DRIVE_ID = 0;
    public static final int BACK_LEFT_DRIVE_ID = 0;
    public static final int BACK_RIGHT_DRIVE_ID = 0;
    
    public static final int FRONT_LEFT_ROTATE_ID = 0;
    public static final int FRONT_RIGHT_ROTATE_ID = 0;
    public static final int BACK_LEFT_ROTATE_ID = 0;
    public static final int BACK_RIGHT_ROTATE_ID = 0;

    public static final int FRONT_LEFT_CANCODER_ID = 0;
    public static final int FRONT_RIGHT_CANCODER_ID = 0;
    public static final int BACK_LEFT_CANCODER_ID = 0;
    public static final int BACK_RIGHT_CANCODER_ID = 0;

    public static final double FL_OFFSET = 0.0;
    public static final double FR_OFFSET = 0.0;
    public static final double BL_OFFSET = 0.0;
    public static final double BR_OFFSET = 0.0;

    public static final double FL_DIAMETER = 0.0;
    public static final double FR_DIAMETER = 0.0;
    public static final double BL_DIAMETER = 0.0;
    public static final double BR_DIAMETER = 0.0;

    public static final double DRIVE_GEAR_RATIO = 0.0;
    public static final double ROTATE_GEAR_RATIO = 0.0;
    
    public static final double ROTATE_POSITION_CONVERSION = (2*Math.PI)/ROTATE_GEAR_RATIO;
    public static final double ROTATE_VELOCITY_CONVERSION = ROTATE_POSITION_CONVERSION/60.0;

    public static final double[] DRIVE_PIDF_VALUES = {0.0, 0.0, 0.0, 0.0};
    public static final double[] ROTATE_PID_VALUES = {0.0, 0.0, 0.0};
  }
}
