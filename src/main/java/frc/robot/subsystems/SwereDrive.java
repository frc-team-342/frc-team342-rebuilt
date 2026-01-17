// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.BACK_LEFT_CANCODER_ID;
import static frc.robot.Constants.DriveConstants.BACK_LEFT_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.BACK_LEFT_ROTATE_ID;
import static frc.robot.Constants.DriveConstants.BACK_RIGHT_CANCODER_ID;
import static frc.robot.Constants.DriveConstants.BACK_RIGHT_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.BACK_RIGHT_ROTATE_ID;
import static frc.robot.Constants.DriveConstants.BL_DIAMETER;
import static frc.robot.Constants.DriveConstants.BL_OFFSET;
import static frc.robot.Constants.DriveConstants.BR_DIAMETER;
import static frc.robot.Constants.DriveConstants.BR_OFFSET;
import static frc.robot.Constants.DriveConstants.FL_DIAMETER;
import static frc.robot.Constants.DriveConstants.FL_OFFSET;
import static frc.robot.Constants.DriveConstants.FRONT_LEFT_CANCODER_ID;
import static frc.robot.Constants.DriveConstants.FRONT_LEFT_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.FRONT_LEFT_ROTATE_ID;
import static frc.robot.Constants.DriveConstants.FRONT_RIGHT_CANCODER_ID;
import static frc.robot.Constants.DriveConstants.FRONT_RIGHT_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.FRONT_RIGHT_ROTATE_ID;
import static frc.robot.Constants.DriveConstants.FR_DIAMETER;
import static frc.robot.Constants.DriveConstants.FR_OFFSET;
import static frc.robot.Constants.DriveConstants.MAX_DRIVE_SPEED;
import static frc.robot.Constants.DriveConstants.MAX_ROTATE_SPEED;
import static frc.robot.Constants.DriveConstants.MIN_DRIVE_SPEED;
import static frc.robot.Constants.DriveConstants.MIN_ROTATE_SPPEED;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwereModule;


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

  /**Returns the modules positions */
  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
      new SwerveModulePosition(frontLeftModule.getDistance(), new Rotation2d(frontLeftModule.getRotateEncoderAngle())),
      new SwerveModulePosition(frontRightModule.getDistance(), new Rotation2d(frontRightModule.getRotateEncoderAngle())),
      new SwerveModulePosition(backLeftModule.getDistance(), new Rotation2d(backLeftModule.getRotateEncoderAngle())),
      new SwerveModulePosition(backRightModule.getDistance(), new Rotation2d(backRightModule.getRotateEncoderAngle()))
    };
  }

  /**Returns if the current alliance is red*/
  public Boolean isRed(){
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  /**Returns the gryo's reading in radians */
  public double gyroRadians(){
    return navx.getAngle() * (Math.PI/180.0);
  }

  /**Returns the robots chassis speeds */
  public ChassisSpeeds getChassisSpeeds(){
    return chassisSpeeds;
  }

  /**Sets each modules chassis speeds to a given chassis speed */
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

  /**Returns the robots current pose */
  public Pose2d getPose2d(){
    return odometry.getPoseMeters();
  }

  /**resets the robots odometry*/
  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(new Rotation2d(gyroRadians()), getModulePositions(), pose);
  }

  /**Resets the robot's pose */
  public void resetPose(Pose2d pose){
    odometry.resetPose(pose);
  }

  /**Stops Swere drive */
  public void stop(){
    frontLeftModule.stop();
    frontRightModule.stop();
    backLeftModule.stop();
    backRightModule.stop();
  }

  /**Drives swere from Joystick inputs*/
  public Command driveWithJoystick(XboxController controller){
    double leftTrigger = controller.getLeftTriggerAxis();
    double speedModifier = MAX_DRIVE_SPEED - (leftTrigger * (MAX_DRIVE_SPEED - MIN_DRIVE_SPEED));
    double rotateModifier = MAX_ROTATE_SPEED - (leftTrigger * (MAX_ROTATE_SPEED - MIN_ROTATE_SPPEED));

    double xSpeed = MathUtil.applyDeadband(controller.getLeftX(), 0.15) * speedModifier;
    double ySpeed = MathUtil.applyDeadband(controller.getLeftY(), 0.15) * speedModifier;
    double rotateSpeed = MathUtil.applyDeadband(controller.getRawAxis(4), 0.15) * rotateModifier;

    chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, -rotateSpeed);

    return Commands.runEnd(() -> drive(chassisSpeeds), () -> {stop();}, this);
  }

  //Puts swere module data
  public void putFrontLeftData(SendableBuilder builder){
    builder.addDoubleProperty(frontLeftModule.getLabel() + "Offset", () -> frontLeftModule.getRawOffset(), null);
    builder.addDoubleProperty(frontLeftModule.getLabel() + "Rotate position", () -> frontLeftModule.getRotateEncoderAngle(), null);
    builder.addDoubleProperty(frontLeftModule.getLabel() + "Rotate Absolute Position", () -> frontLeftModule.getRotateInRadian(), null);
    builder.addDoubleProperty(frontLeftModule.getLabel() + "Drive Velocity", () -> frontLeftModule.getVelocity(), null);
    builder.addDoubleProperty(frontLeftModule.getLabel() + "Drive Distance", () -> frontLeftModule.getDistance(), null);
  }

  public void putFrontRightData(SendableBuilder builder){
    builder.addDoubleProperty(frontRightModule.getLabel() + "Offset", () -> frontRightModule.getRawOffset(), null);
    builder.addDoubleProperty(frontRightModule.getLabel() + "Rotate position", () -> frontRightModule.getRotateEncoderAngle(), null);
    builder.addDoubleProperty(frontRightModule.getLabel() + "Rotate Absolute Position", () -> frontRightModule.getRotateInRadian(), null);
    builder.addDoubleProperty(frontRightModule.getLabel() + "Drive Velocity", () -> frontRightModule.getVelocity(), null);
  }

  public void putBackLeftData(SendableBuilder builder){
    builder.addDoubleProperty(backRightModule.getLabel() + "Offset", () -> backRightModule.getRawOffset(), null);
    builder.addDoubleProperty(backRightModule.getLabel() + "Rotate position", () -> backRightModule.getRotateEncoderAngle(), null);
    builder.addDoubleProperty(backRightModule.getLabel() + "Rotate Absolute Position", () -> backRightModule.getRotateInRadian(), null);
    builder.addDoubleProperty(backRightModule.getLabel() + "Drive Velocity", () -> backRightModule.getVelocity(), null);
  }

  public void putBackRightData(SendableBuilder builder){
    builder.addDoubleProperty(backLeftModule.getLabel() + "Offset", () -> backLeftModule.getRawOffset(), null);
    builder.addDoubleProperty(backLeftModule.getLabel() + "Rotate position", () -> backLeftModule.getRotateEncoderAngle(), null);
    builder.addDoubleProperty(backLeftModule.getLabel() + "Rotate Absolute Position", () -> backLeftModule.getRotateInRadian(), null);
    builder.addDoubleProperty(backLeftModule.getLabel() + "Drive Velocity", () -> backLeftModule.getVelocity(), null);
  }

  //Puts swere drive data
  public void initSendable(SendableBuilder builder){
    putFrontLeftData(builder);
    putFrontRightData(builder);
    putBackLeftData(builder);
    putBackRightData(builder);

    builder.addBooleanProperty("Field Oriented", () -> fieldOriented, null);

    builder.addDoubleProperty("Gyro Reading", () -> gyroRadians(), null);

    builder.addDoubleProperty("Pose2d X", () -> odometry.getPoseMeters().getX(), null);
    builder.addDoubleProperty("Pose2d Y", () -> odometry.getPoseMeters().getY(), null);

    builder.addDoubleProperty("Rotation", () -> odometry.getPoseMeters().getRotation().getRadians(), null);

    builder.addBooleanProperty("Red?", () -> isRed(), null);

    builder.addDoubleProperty("Match Time", () -> DriverStation.getMatchTime(), null);
  }

  //Vision code will be added later (maybe)

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
