// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.text.ParseException;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwereModule;
import static frc.robot.Constants.DriveConstants.*;


public class SwereDrive extends SubsystemBase {
  private SwerveDriveKinematics kinematics;
  private ChassisSpeeds chassisSpeeds;
  public SwerveDriveOdometry odometry;

  private SwereModule frontLeftModule;
  private SwereModule frontRightModule;
  private SwereModule backLeftModule;
  private SwereModule backRightModule;

  private SwerveModuleState[] swerveModuleStates;
  private SwerveModulePosition[] swerveModulePositions;

  private Supplier<Pose2d> poseSupplier;
  private Consumer<Pose2d> resetPoseConsumer;

  private Supplier<ChassisSpeeds> chassisSpeedSupplier;
  private Consumer<ChassisSpeeds> robotRelativeOutput;

  private BooleanSupplier shouldFlipSupplier;
  private RobotConfig config;
  private Field2d field;

  private AHRS navx;

  private boolean fieldOriented;
  private boolean isRed;

  private int tag;
  

  /** Creates a new SwereDrive. */
  public SwereDrive() {
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    isRed = isRed();
    

    frontLeftModule = new SwereModule(
      FRONT_LEFT_DRIVE_ID,
      FRONT_LEFT_ROTATE_ID, 
      FRONT_LEFT_CANCODER_ID, 
      false, 
      true, 
      FL_OFFSET, 
      FL_DIAMETER, 
      "Front Left");

    frontRightModule = new SwereModule(
      FRONT_RIGHT_DRIVE_ID,
      FRONT_RIGHT_ROTATE_ID, 
      FRONT_RIGHT_CANCODER_ID, 
      false, 
      true, 
      FR_OFFSET, 
      FR_DIAMETER, 
      "Front Right");

    backLeftModule = new SwereModule(
      BACK_LEFT_DRIVE_ID,
      BACK_LEFT_ROTATE_ID, 
      BACK_LEFT_CANCODER_ID, 
      false, 
      true, 
      BL_OFFSET, 
      BL_DIAMETER, 
      "Back Left");

    backRightModule = new SwereModule(
      BACK_RIGHT_DRIVE_ID,
      BACK_RIGHT_ROTATE_ID, 
      BACK_RIGHT_CANCODER_ID, 
      false, 
      true, 
      BR_OFFSET, 
      BR_DIAMETER, 
      "Back Right");

      
      field = new Field2d();

      kinematics = new SwerveDriveKinematics(
        new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0))
      );

      navx = new AHRS(AHRS.NavXComType.kUSB1);

      odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyroRadians()), getModulePositions());

      fieldOriented = false;
      
      poseSupplier = () -> getPose2d();
      resetPoseConsumer = pose -> resetOdometry(pose);
      robotRelativeOutput = chassisSpeeds -> drive(chassisSpeeds);
      chassisSpeedSupplier = () -> getChassisSpeeds();
      shouldFlipSupplier = () -> isRed();

      // try {
      //     config = RobotConfig.fromGUISettings();
      //   } catch (IOException e) {
      //     e.printStackTrace();
      //   } catch (ParseException e) {
      //     e.printStackTrace();
      //   }    

  }

  public Boolean isRed(){
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  public double gyroRadians(){
    return navx.getAngle() * (Math.PI/180.0);
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
      new SwerveModulePosition(frontLeftModule.getDistance(), new Rotation2d(frontLeftModule.getRotateEncoderAngle())),
      new SwerveModulePosition(frontRightModule.getDistance(), new Rotation2d(frontRightModule.getRotateEncoderAngle())),
      new SwerveModulePosition(backLeftModule.getDistance(), new Rotation2d(backLeftModule.getRotateEncoderAngle())),
      new SwerveModulePosition(backRightModule.getDistance(), new Rotation2d(backRightModule.getRotateEncoderAngle()))
    };
  }

  public ChassisSpeeds getChassisSpeeds(){
    return chassisSpeeds;
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    if(fieldOriented)
      chassisSpeeds = chassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, new Rotation2d(gyroRadians()));
    SwerveModuleState swerveModuleStates[] = kinematics.toWheelSpeeds(chassisSpeeds);

    frontLeftModule.setState(swerveModuleStates[0]);
    frontRightModule.setState(swerveModuleStates[1]);
    backLeftModule.setState(swerveModuleStates[2]);
    backRightModule.setState(swerveModuleStates[3]);

    this.chassisSpeeds = chassisSpeeds;
  }

  public Pose2d getPose2d(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(new Rotation2d(gyroRadians()), getModulePositions(), pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
