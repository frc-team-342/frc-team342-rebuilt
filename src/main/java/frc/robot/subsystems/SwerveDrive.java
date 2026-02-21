// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.PhotonVision;

import java.io.IOException;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import javax.print.attribute.standard.MediaSize.NA;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.core.filter.FilteringGeneratorDelegate;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveWithJoystick;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends SubsystemBase {

  private PhotonVision photonVision;

  private SwerveDriveKinematics kinematics;
  public SwerveDriveOdometry odometry;
  private AHRS NavX;

  private ChassisSpeeds chassisSpeeds;
  
  private boolean fieldOriented; 
  private boolean slowMode;
  private boolean redSide;
  
  private int tag;
  
  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule; 

  private Supplier<Pose2d> poseSupplier;
  private Consumer<Pose2d> resetPoseConsumer;
  private Consumer<ChassisSpeeds> robotRelativeOutput;
  private Supplier<ChassisSpeeds> chasisSpeedSupplier;
  private BooleanSupplier shouldFlipSupplier;
  private RobotConfig config;
  private Field2d field;
  
    SwerveModuleState[] swerveModuleStates;
    SwerveModulePosition[] swerveModulePositions;
  
    /** Creates a new SwerveDrive. */
    public SwerveDrive(PhotonVision photonVision) {
      this.photonVision = photonVision;

        chassisSpeeds = new ChassisSpeeds(0,0,0);

        redSide = isRed();

        frontLeftModule = new SwerveModule(
          DriveConstants.FRONT_LEFT_DRIVE_ID, 
          DriveConstants.FRONT_LEFT_ROTATE_ID, 
          DriveConstants.FRONT_LEFT_CANCODER_ID, 

          false, true, 
          "FL",
          DriveConstants.FL_DIAMETER
          );
  
          frontRightModule = new SwerveModule(
          DriveConstants.FRONT_RIGHT_DRIVE_ID, 
          DriveConstants.FRONT_RIGHT_ROTATE_ID, 
          DriveConstants.FRONT_LEFT_CANCODER_ID, 

          false, true, 
          "FR",
          DriveConstants.FR_DIAMETER

          );
  
          backLeftModule = new SwerveModule(
          DriveConstants.BACK_LEFT_DRIVE_ID, 
          DriveConstants.BACK_LEFT_ROTATE_ID, 
          DriveConstants.BACK_LEFT_CANCODER_ID, 

          false, true, 
          "BL",
          DriveConstants.BL_DIAMETER

          );
  
          backRightModule = new SwerveModule(
          DriveConstants.BACK_RIGHT_DRIVE_ID, 
          DriveConstants.BACK_RIGHT_ROTATE_ID, 
          DriveConstants.BACK_RIGHT_CANCODER_ID,

          false, true, 
          "BR",
          DriveConstants.BR_DIAMETER

          );
  
          field = new Field2d();

        /* Initalizes Kinematics */
        kinematics = new SwerveDriveKinematics(
  
          /*Front Left */ new Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(10.375)),
          /*Front Right */ new Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(-10.375)),
          /*Back Left */ new Translation2d(Units.inchesToMeters(-10.375), Units.inchesToMeters(10.375)),
          /*Back Right */ new Translation2d(Units.inchesToMeters(-10.375), Units.inchesToMeters(-10.375))
  
        );
  
        /* Initalize NavX (Gyro) */
        NavX = new AHRS(AHRS.NavXComType.kUSB1);
  
        /* Initalizes Odometry */
        odometry = new SwerveDriveOdometry( 
  
          kinematics, 
          new Rotation2d(gyroRad()),
          getCurrentSwerveModulePositions()
  
          );

          odometry.resetPose(new Pose2d(0,0,new Rotation2d(0)));
      
        fieldOriented = false;
        slowMode = false;

      poseSupplier = () -> getPose2d();
      resetPoseConsumer = pose -> setPose(pose);
      robotRelativeOutput = chassisSpeeds -> drive(chassisSpeeds);
      chasisSpeedSupplier = () -> getChassisSpeeds();
      shouldFlipSupplier = () -> isRed();
                       
        try {
          config = RobotConfig.fromGUISettings();
        } catch(Exception e){
          e.printStackTrace();
        }


        field = new Field2d();
  
          configureAutoBuilder();
    }

    /**Checks if the driver station is set to red alliance.
     * 
     * @return {@code true} if driver station is set to red, {@code false} otherwise.
     */
      public Boolean isRed(){
        var alliance = DriverStation.getAlliance();
        return alliance.get() == DriverStation.Alliance.Red;
      }
  
      /**Toggles field oriented.
       * 
       */
      public void toggleFieldOriented (){
        
        fieldOriented = !fieldOriented;
  
      }
  
      /**Toggles slow mode.
       * 
       */
      public void toggleSlowMode() {
      slowMode = !slowMode;
      }

      /**Checks if the robot is in slow mode.
       * 
       * @return {@code true} if the robot is in slow mode, {@code false} otherwise.
       */
      public boolean getSlowMode() {
        return slowMode;
      }
  
      /**Gets the chassis speeds.
       * 
       * @return The chassis speeds.
       */
      public ChassisSpeeds getChassisSpeeds(){
        return chassisSpeeds;
      }
  
        /**This drive method takes the values from the chassisspeeds and
         * applys it to each individual Module using the "SetState" Method created in SwerveModule.
         * 
         * @param chassisSpeeds The ChassisSpeeds to pull values from.
         */
        public void drive(ChassisSpeeds chassisSpeeds) {
  
          /* When Field Oriented is True, passes the chassis speed and the Gryo's current angle through "fromFieldRelativeSpeeds",
           before passing it through the rest of the drive Method */
  
          if (fieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, new Rotation2d(gyroRad())); //NAVX USED TO BE HERE
  
          }

          SwerveModuleState swerveModuleStates[] = kinematics.toWheelSpeeds(chassisSpeeds);

  
          frontLeftModule.setState(swerveModuleStates[0]);
          frontRightModule.setState(swerveModuleStates[1]);
          backLeftModule.setState(swerveModuleStates[2]);
          backRightModule.setState(swerveModuleStates[3]);

          this.chassisSpeeds = chassisSpeeds;

    }
  
        /**Spins the wheels.
         * 
         */
        public void testDrive(){
  
          ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(1), Units.inchesToMeters(0), Units.degreesToRadians(0));
  
          SwerveModuleState[] swerveModuleStates = kinematics.toWheelSpeeds(testSpeeds);
  
          frontLeftModule.setState(swerveModuleStates[0]);
          frontRightModule.setState(swerveModuleStates[1]);
          backLeftModule.setState(swerveModuleStates[2]);
          backRightModule.setState(swerveModuleStates[3]);
        }

        /**Gets the current module positions.
         * 
         * @return The current module positions.
         */
        public SwerveModulePosition[] getCurrentSwerveModulePositions(){
          return new SwerveModulePosition[]{
  
              new SwerveModulePosition(frontLeftModule.getDistance(), new Rotation2d(frontLeftModule.getRotateEncoderPosition())), // Front left
              new SwerveModulePosition(frontRightModule.getDistance(), new Rotation2d(frontRightModule.getRotateEncoderPosition())), // Front Right
              new SwerveModulePosition(backLeftModule.getDistance(), new Rotation2d(backLeftModule.getRotateEncoderPosition())), // Back Left
              new SwerveModulePosition(backRightModule.getDistance(), new Rotation2d(backRightModule.getRotateEncoderPosition())) // Back Right
  
          };
        } 

      /**Stops all modules.
       * 
       */
      public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    /**Gets the pose2d of the robot.
     * 
     * @return The pose2d of the robot.
     */
    public Pose2d getPose2d(){
      return odometry.getPoseMeters();
    }

    /**Resets the odometry. This sets everything to 0.
     * 
     */
    public void resetOdometry(){
      SwerveModulePosition[] blankModulePositions = new SwerveModulePosition[4];
      Arrays.fill(blankModulePositions, new SwerveModulePosition());

      odometry.resetPosition(new Rotation2d(), swerveModulePositions, new Pose2d());
    }

    /**Updates the odometry using the encoders.
     * 
     */
    public void updateOdometry() {
      odometry.update(new Rotation2d(gyroRad()), getCurrentSwerveModulePositions());
    }

    /**Updates the odometry using vision.
     * 
     */
    public void updateOdometryWithVision() {
      odometry.resetPosition(new Rotation2d(gyroRad()), getCurrentSwerveModulePositions(), photonVision.getRobotPose2d().get());
    }
    
    /**Gets the NavX.
     * 
     * @return The NavX.
     */
    public AHRS getGyro(){
      return NavX;
    }

    /**Gets the reading of the NavX in radians.
     * 
     * @return The yaw in radians.
     */
    public double gyroRad(){
      return NavX.getYaw() * Math.PI/180;
    }

    /**Resets the pose2d of the robot. This sets everything to 0.
     * 
     */
    public void resetPose(){
      odometry.resetPose(new Pose2d());
    }

    /**Sets the pose2d of the robot to the given pose2d.
     * 
     * @param pose The pose2d to set the pose2d to.
     */
    public void setPose(Pose2d pose) {
      odometry.resetPose(pose);
    }

    /**Updates the pose2d using vision.
     * 
     */
    public void updatePoseWithVision() {
      odometry.resetPose(photonVision.getRobotPose2d().get());
    }

    /**Spins the back right rotate motor.
     * 
     */
    public void spinBR(){
      backRightModule.spinRotate();
    }

    /**Configures the auto builder for PathPlanner.
     * 
     */
    public void configureAutoBuilder() {
      AutoBuilder.configure(
        poseSupplier, 
        resetPoseConsumer, 
        chasisSpeedSupplier, 
        robotRelativeOutput, 
        DriveConstants.PATH_CONFIG_CONTROLLER, 
        config, 
        shouldFlipSupplier,
        this
        );
    }
    

    /**Puts the front left module values onto Elastic.
     * 
     * @param sendableBuilder The SendableBuilder to use.
     */
    public void putFrontLeftValues(SendableBuilder sendableBuilder){
      sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Rotate Encoder(Radians): " , ()-> frontLeftModule.getRotateEncoderPosition(), null);
      sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Rotate Setpoint", () -> frontLeftModule.getRotateSetpoint(), null);
      sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Absolute Position " , ()-> frontLeftModule.absoluteRotatePosition(), null);
      sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + "Velocity", () -> frontLeftModule.getDriveVelocity(), null);
      sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " drive setpoint", () -> (frontLeftModule.getDriveSetpoint()), null);
      if(swerveModuleStates != null)
        sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Analog Offest " , ()-> swerveModuleStates[0].angle.getRadians(), null);

    }

    /**Puts the front right module values onto Elastic.
     * 
     * @param sendableBuilder The SendableBuilder to use.
     */
    public void putFrontRightValues(SendableBuilder sendableBuilder){
      sendableBuilder.addDoubleProperty(frontRightModule.printLabel() + " Rotate Encoder(Radians): " , ()-> frontRightModule.getRotateEncoderPosition(), null);
      sendableBuilder.addDoubleProperty(frontRightModule.printLabel() + " Rotate Setpoint", () -> frontRightModule.getRotateSetpoint(), null);
      sendableBuilder.addDoubleProperty(frontRightModule.printLabel() + " Absolute Position " , ()-> frontRightModule.absoluteRotatePosition(), null);
      sendableBuilder.addDoubleProperty(frontRightModule.printLabel() + "Velocity", () -> frontRightModule.getDriveVelocity(), null);
      sendableBuilder.addDoubleProperty(frontRightModule.printLabel() + " drive setpoint", () -> (frontRightModule.getDriveSetpoint()), null);
      if(swerveModuleStates != null)
        sendableBuilder.addDoubleProperty(frontRightModule.printLabel() + " Analog Offest " , ()-> swerveModuleStates[1].angle.getRadians(), null);
    }

    /**Puts the back left module values onto Elastic.
     * 
     * @param sendableBuilder The SendableBuilder to use.
     */
    public void putBackLeftModule(SendableBuilder sendableBuilder){
      sendableBuilder.addDoubleProperty(backLeftModule.printLabel() + " Rotate Encoder(Radians): " , ()-> backLeftModule.getRotateEncoderPosition(), null);
      sendableBuilder.addDoubleProperty(backLeftModule.printLabel() + " Rotate Setpoint", () -> backLeftModule.getRotateSetpoint(), null);
      sendableBuilder.addDoubleProperty(backLeftModule.printLabel() + " Absoulete Position " , ()-> backLeftModule.absoluteRotatePosition(), null);
      sendableBuilder.addDoubleProperty(backLeftModule.printLabel() + "Velocity", () -> backLeftModule.getDriveVelocity(), null);
      sendableBuilder.addDoubleProperty(backLeftModule.printLabel() + " drive setpoint", () -> (backLeftModule.getDriveSetpoint()), null);
      if(swerveModuleStates != null)
        sendableBuilder.addDoubleProperty(backLeftModule.printLabel() + " Analog Offest " , ()-> swerveModuleStates[2].angle.getRadians(), null);

    }

    /**Puts the back right module values onto Elastic.
     * 
     * @param sendableBuilder The SendableBuilder to use.
     */
    public void putBackRightModule(SendableBuilder sendableBuilder){
      sendableBuilder.addDoubleProperty(backRightModule.printLabel() + " Rotate Encoder(Radians): " , ()-> backRightModule.getRotateEncoderPosition(), null);
      sendableBuilder.addDoubleProperty(backRightModule.printLabel() + " Rotate Setpoint", () -> backRightModule.getRotateSetpoint(), null);
      sendableBuilder.addDoubleProperty(backRightModule.printLabel() + " Absoulete Position " , ()-> backRightModule.absoluteRotatePosition(), null);
      sendableBuilder.addDoubleProperty(backRightModule.printLabel() + "Velocity", () -> backRightModule.getDriveVelocity(), null);
      sendableBuilder.addDoubleProperty(backRightModule.printLabel() + " drive setpoint", () -> (backRightModule.getDriveSetpoint()), null);
      if(swerveModuleStates != null)
        sendableBuilder.addDoubleProperty(backRightModule.printLabel() + " Analog Offest " , ()-> swerveModuleStates[3].angle.getRadians(), null);
    }

    /**Puts all swerve drive values onto Elastic.
     * 
     */
  @Override 
  public void initSendable(SendableBuilder sendableBuilder){
    putFrontLeftValues(sendableBuilder);
    putFrontRightValues(sendableBuilder);
    putBackLeftModule(sendableBuilder);
    putBackRightModule(sendableBuilder);
    sendableBuilder.addBooleanProperty("Field Orienated", ()-> fieldOriented, null);
    sendableBuilder.addBooleanProperty("Slow Mode", ()-> slowMode, null);

    sendableBuilder.addDoubleProperty("Gyro Reading", ()-> gyroRad(), null); //NAVX USED TO BE HERE
    sendableBuilder.addDoubleProperty("Raw Gyro Reading", ()-> NavX.getYaw(), null); //NAVX USED TO BE HERE

    sendableBuilder.addDoubleProperty("FL Distance Travelled", ()-> frontLeftModule.getDistance(), null);
    sendableBuilder.addDoubleProperty("FL Velocity", ()-> frontLeftModule.getDriveVelocity(), null);

    sendableBuilder.addDoubleProperty("Pose2d  X", () ->  odometry.getPoseMeters().getX(), null);
    sendableBuilder.addDoubleProperty("Pose2d  Y", () ->  odometry.getPoseMeters().getY(), null);

    sendableBuilder.addDoubleProperty("Rotations", () ->  odometry.getPoseMeters().getRotation().getRadians(), null);

    sendableBuilder.addDoubleProperty("Chassis speeds, X", () -> getChassisSpeeds().vxMetersPerSecond, null);
    sendableBuilder.addDoubleProperty("Chassis speeds, Y", () -> getChassisSpeeds().vyMetersPerSecond, null);
    sendableBuilder.addDoubleProperty("Chassis speeds, rotation", () -> getChassisSpeeds().omegaRadiansPerSecond, null);

    sendableBuilder.addBooleanProperty("Am I red?", () -> redSide, null);

    sendableBuilder.addDoubleProperty("Match Time", () -> DriverStation.getMatchTime(), null);
    
    SmartDashboard.putData(field);

  }
      
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Updates the odometry every run

    if(photonVision.tagIsPresentAcrossAllCameras()) {
      updateOdometryWithVision();
    }else{
      updateOdometry();
    }

    field.setRobotPose(odometry.getPoseMeters());
  }
}