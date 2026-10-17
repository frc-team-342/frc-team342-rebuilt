// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class SwerveDrive extends SubsystemBase {

  /*
   * The PhotonVision subsystem is used in the SwerveDrive subsystem for pose estimation with the SwerveDrivePoseEstimator class.
   */
  private PhotonVision photonVision;

  /*
   * The SwerveDriveKinematics class is used to convert robot speeds into individual wheel speeds and angles,
   * individual speeds and angles into robot speeds, and provides optimization methods for wheel rotation.
   */
  private SwerveDriveKinematics kinematics;

  /*
   * The SwerveDriveOdometry class keeps track of the robot position on the field using SwerveModule encoder data and
   * gyroscope angles.
   */
  public SwerveDriveOdometry odometry;

  /*
   * The SwerveDrivePoseEstimator updates the robot's field relative pose (pose2d) using odometry data and
   * latency-compensated vision readings.
   */
  private SwerveDrivePoseEstimator swervePoseEstimator;

  /*
   * The gyroscope keeps track of the angle of the robot.
   */
  private AHRS NavX;

  /*
   * Instance variable to hold the ChassisSpeeds (linear and angular velocity) of the robot chassis. This
   * is used for kinematics and odometry.
   */
  private ChassisSpeeds chassisSpeeds;
  
  /*
   * Booleans for different conditions:
   * 
   * fieldOriented = false for robot-relative (forward changes based on where the front of the robot is), true for
   * field-relative (forward stays the same no matter the orientation of the robot)
   * 
   * slowMode = false for regular linear and angular velocities, true for slowed-down linear and angular velocities
   * 
   * redSide = false if on blue alliance, true if on red alliance
   * 
   * driveAssist is currently NOT in use.
   * driveAssist = false for rotation lock off, true for rotation lock on
   * 
   * shootWhileMoving is currently NOT in use.
   * shootWhileMoving = false for regular linear and angular velocities, true for slowed-down linear and angular
   * velocities that are adjusted for shooting on the move
   */
  private boolean fieldOriented; 
  private boolean slowMode;
  private boolean redSide;
  private boolean driveAssist;
  private boolean shootWhileMoving;
  
  // private Pose2d robotPose;

  /*
   * An array containing all 4 swerve modules
   */
  private SwerveModule[] swerveModules;

  /*
   * All 4 swerve modules
   */
  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  /*
   * Suppliers for pathplanning softwares
   */
  private Supplier<Pose2d> poseSupplier;
  private Consumer<Pose2d> resetPoseConsumer;
  private Consumer<ChassisSpeeds> robotRelativeOutput;
  private Supplier<ChassisSpeeds> chasisSpeedSupplier;
  private BooleanSupplier shouldFlipSupplier;

  /*
   * Variable for PathPlanner UI config
   */
  private RobotConfig config;

  /*
   * Image of the field; robot pose can be supplied to the field, which will
   * put the robot on the image of the field
   */
  private Field2d field;

  /*
   * SysIdRoutine for SwerveDrive; this can be used to determine PID values for SwerveDrive
   */
  private SysIdRoutine swerveSysIdRoutine;
  
  /*
   * Arrays holding the states and positions of all 4 swerve modules.
   */
    SwerveModuleState[] swerveModuleStates;
    SwerveModulePosition[] swerveModulePositions;
  
    /** Creates a new SwerveDrive. */
  public SwerveDrive(PhotonVision photonVision) {
    /*
     * Remember to always use the "this" keyword to instantiate subsystems when instantiating them
     * outside of RobotContainer. This is to prevent multiple instances of the same subsystem,
     * which would throw an error.
     */
    this.photonVision = photonVision;

    /*
     * Instantiate chassisSpeeds with 0 for all velocities; the robot should start
     * completely still.
     */
    chassisSpeeds = new ChassisSpeeds(0,0,0);

    redSide = isRed();

    /*
     * Instantiate all 4 swerve modules; follow parameters from the SwerveModule class
     */
    frontLeftModule = new SwerveModule(
      DriveConstants.FRONT_LEFT_DRIVE_ID, 
      DriveConstants.FRONT_LEFT_ROTATE_ID, 
      DriveConstants.FRONT_LEFT_CANCODER_ID, 
      false, 
      false, 
      "FL",
      DriveConstants.FL_DIAMETER
    );
  
          frontRightModule = new SwerveModule(
          DriveConstants.FRONT_RIGHT_DRIVE_ID, 
          DriveConstants.FRONT_RIGHT_ROTATE_ID, 
          DriveConstants.FRONT_RIGHT_CANCODER_ID, 

          false, true, 
          "FR",
          DriveConstants.FR_DIAMETER

          );
  
    backLeftModule = new SwerveModule(
      DriveConstants.BACK_LEFT_DRIVE_ID, 
      DriveConstants.BACK_LEFT_ROTATE_ID, 
      DriveConstants.BACK_LEFT_CANCODER_ID, 
      false,
      false, 
      "BL",
      DriveConstants.BL_DIAMETER
    );
  
    backRightModule = new SwerveModule(
      DriveConstants.BACK_RIGHT_DRIVE_ID, 
      DriveConstants.BACK_RIGHT_ROTATE_ID, 
      DriveConstants.BACK_RIGHT_CANCODER_ID,
      false, 
      true, 
      "BR",
      DriveConstants.BR_DIAMETER
    );

    /*
     * Instantiate array for swerve modules, and fill the array
     */
    swerveModules = new SwerveModule[4];
    swerveModules[0] = frontLeftModule;
    swerveModules[1] = frontRightModule;
    swerveModules[2] = backLeftModule;
    swerveModules[3] = backRightModule;

    /* Initalizes Kinematics */
    kinematics = new SwerveDriveKinematics(
      /*Front Left */ new Translation2d(0.28575, 0.28575), //11.25 inches
      /*Front Right */ new Translation2d(0.28575, -0.28575),
      /*Back Left */ new Translation2d(-0.28575, 0.28575),
      /*Back Right */ new Translation2d(-0.28575, -0.28575)
    );
  
        /* Initalize NavX (Gyro) */
    NavX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    NavX.setAngleAdjustment(74.68000244140625);
      
    /*
     * All of these booleans should be false whenever the robot is first started
     */
    fieldOriented = false;
    slowMode = false;
    shootWhileMoving = false;

    /*
     * Set all suppliers/consumers
     */
    poseSupplier = () -> getPose2d();
    resetPoseConsumer = pose -> setPose(pose);
    robotRelativeOutput = chassisSpeeds -> drive(chassisSpeeds);
    chasisSpeedSupplier = () -> getChassisSpeeds();
    shouldFlipSupplier = () -> isRed();

    /*
     * Try-catch statements allow you to run code that could potentially throw a runtime error without
     * crashing the entire program. It runs the code in the try statement. If the error given to the
     * catch statement occurs, the code in the catch statement occurs.
     */
    try {
      config = RobotConfig.fromGUISettings();
    } 
    catch(Exception e){
      e.printStackTrace();
    }

    /*
     * Instantiate and fill the array of swerve module positions
     */
    SwerveModulePosition[] initialModulePositions = {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };

    field = new Field2d();
    
    /*
     * We don't want the rotation of the robot to be locked on startup
     */
    driveAssist = false;
  
    /*
     * Inputs all the necessary parameters into the AutoBuilder.configure() method
     */
    configureAutoBuilder();

    /*
     * Instantiating swerveSysIdRoutine
     */
    swerveSysIdRoutine = new SysIdRoutine(
      new Config(
        Volts.of(1).per(Second),
        Volts.of(3),
        Seconds.of(5)
      ),
      new Mechanism(
        (volts) -> runCharacterization(volts.in(Volts)), null, this)
    );

    /*
     * Instantiating swervePoseEstimator
     */
    swervePoseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      new Rotation2d(gyroRad()),
      initialModulePositions,
      new Pose2d()
      // VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10)),
      // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

    /*
     * Syncs the encoders, which allows the swerve modules to auto align to their zero on robot enable.
     */
    syncEncoders();

    /*
     * Zero the states of all 4 modules; none of the modules should be moving when the robot first starts.
     */
    frontLeftModule.setState(new SwerveModuleState(0, new Rotation2d(0)));
    frontRightModule.setState(new SwerveModuleState(0, new Rotation2d(0)));
    backLeftModule.setState(new SwerveModuleState(0, new Rotation2d(0)));
    backRightModule.setState(new SwerveModuleState(0, new Rotation2d(0)));
  }

  /**Sets the state of every swerve module to the given SwerveModuleState.
   * 
   * @param state The desired SwerveModuleState.
   */
  public void setState(SwerveModuleState state) {
    frontLeftModule.setState(state);
    frontRightModule.setState(state);
    backLeftModule.setState(state);
    backRightModule.setState(state);
  }

  /**Runs all the drive motors at the given output while keeping all the rotate motors
   * at zero.
   * 
   * @param output The desired drive motor output.
   */
  public void runCharacterization(double output) {
    for(int i = 0; i < 4; i++) {
      swerveModules[i].runCharacterization(output);
    }
  }

  /**Returns a command that runs the quasistatic test for swerveSysIdRoutine.
   * 
   * @param direction The desired direction of the motors.
   * @return A command that runs the quasistatic test for swerveSysIdRoutine.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
      .withTimeout(1.0)
      .andThen(swerveSysIdRoutine.quasistatic(direction)).withTimeout(3.0
    );
  }

  /**Returns a command that runs the dynamic test for swerveSysIdRoutine.
   * 
   * @param direction The desired direction of the motors.
   * @return A command that runs the dynamic test for swerveSysIdRoutine.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
      .withTimeout(1.0)
      .andThen(swerveSysIdRoutine.dynamic(direction)).withTimeout(3.0
    );
  }

  /**Returns a command that runs two quasistatic tests and two dynamic tests for swerveSysIdRoutine.
   * 
   * @return A command that runs two quasistatic tests and two dynamic tests for swerveSysIdRoutine.
   */
  public Command runSwerveSysID() {
    return Commands.sequence(
      swerveSysIdRoutine
        .quasistatic(Direction.kForward)
        .withTimeout(3),
      swerveSysIdRoutine
        .quasistatic(Direction.kReverse)
        .withTimeout(3),
      swerveSysIdRoutine
        .dynamic(Direction.kForward)
        .withTimeout(3),
      swerveSysIdRoutine
        .dynamic(Direction.kReverse)
        .withTimeout(3)
    );
  }

  /**Checks if the driver station is set to red alliance.
  * @return {@code true} if driver station is set to red, {@code false} otherwise.
  */
  public Boolean isRed(){
    var alliance = DriverStation.getAlliance();
    return alliance.get() == DriverStation.Alliance.Red;
  }

  /**Toggles field oriented*/
  public void toggleFieldOriented (){ 
      fieldOriented = !fieldOriented;
  }
  
  /**Toggles slow mode.*/
  public void toggleSlowMode() {
    slowMode = !slowMode;
  }

  /**Toggles shoot on the move mode */
  public void toggleShootWhileMoving() {
    shootWhileMoving = !shootWhileMoving;
  }

  /**Checks if the robot is in slow mode.
  * @return {@code true} if the robot is in slow mode, {@code false} otherwise.
  */
  public boolean getSlowMode() {
    return slowMode;
  }

  /**Checks if the robot is in shoot on the move mode.
   * 
   * @return {@code true} if the robot is in shoot on the move mode, {@code false} otherwise.
   */
  public boolean getShootWhileMoving() {
    return shootWhileMoving;
  }

  /**Checks if drive assist mode is on.
   * 
   * @return {@code true} if drive assist mode is on, {@code false} otherwise.
   */
  public boolean getDriveAssist(){
    return driveAssist;
  }

  /**Toggles drive assist mode */
  public void toggleDriveAssist(){
    this.driveAssist = !driveAssist;
  }
  
  /**Gets the chassis speeds.
  * @return The chassis speeds.
  */
  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState()
    );
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
    
    /*
     * Use .toWheelSpeeds() to convert chassisSpeeds into wheel speeds
     */
    SwerveModuleState swerveModuleStates[] = kinematics.toWheelSpeeds(chassisSpeeds);
  
    /*
     * Using the converted wheel speeds, use .setState() on each swerve module to set
     * the state of each module.
     */
    frontLeftModule.setState(swerveModuleStates[0]);
    frontRightModule.setState(swerveModuleStates[1]);
    backLeftModule.setState(swerveModuleStates[2]);
    backRightModule.setState(swerveModuleStates[3]);

    /*
     * Sets the instance variable created to hold the current ChassisSpeeds being used to
     * the given ChassisSpeeds.
     */
    this.chassisSpeeds = chassisSpeeds;
  }

  /*
   * The testDrive() method is used to check if the swerve modules are taking inputs correctly.
   */
  
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
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**Resets the odometry. This sets everything to 0.
   * 
   */
  public void resetOdometry(){
    //Creates and fills an array of SwerveModulePosition with 4 blank module positions.
    SwerveModulePosition[] blankModulePositions = new SwerveModulePosition[4];
    Arrays.fill(blankModulePositions, new SwerveModulePosition());

    //Sets the position of the robot to the origin
    swervePoseEstimator.resetPosition(new Rotation2d(), swerveModulePositions, new Pose2d());
  }

  /**Updates the odometry using the encoders.
   * 
   */
  public void updateOdometry() {
    swervePoseEstimator.update(new Rotation2d(gyroRad()), getCurrentSwerveModulePositions());
  }

  /**Updates the odometry using vision.
   * 
   */
  public void updateOdometryWithVision() {
    /*
     * In order to compensate for latency, the timestamp of the vision reading needs to be provided.
     * This can be obtained using Timer.getFPGATimestamp() or by using a getter method to get the
     * timestamp of the vision reading from the estimated pose provided by PhotonPoseEstimator
     * (look at PhotonVision subsystem/Camera class for more information on latter method).
     */
    swervePoseEstimator.addVisionMeasurement(photonVision.getRobotPose2d().get(), Timer.getFPGATimestamp());
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
   * @return The angle in radians.
   */
  public double  gyroRad(){
    return (NavX.getAngle() * Math.PI/180);
  }

  /**Resets the gyro yaw to zero.
   * 
   */
  public void resetNavX() {
    NavX.reset();
  }

  /**Resets the pose2d of the robot. This sets everything to 0.
   * 
   */
  public void resetPose(){
    swervePoseEstimator.resetPose(new Pose2d());
  }

  /**Sets the pose2d of the robot to the given pose2d.
   * 
   * @param pose The pose2d to set the pose2d to.
   */
  public void setPose(Pose2d pose) {
    swervePoseEstimator.resetPose(pose);
  }

  /**Spins the back right rotate motor.
   * 
   */
  public void spinBR(){
    backRightModule.spinRotate();
  }

  /**Sets the voltage of all 4 drive motors.
   * 
   * @param volts The desired voltage.
   */
  public void setDriveVoltage(double volts) {
    frontLeftModule.setDriveVoltage(volts);
    frontRightModule.setDriveVoltage(volts);
    backLeftModule.setDriveVoltage(volts);
    backRightModule.setDriveVoltage(volts);
  }

  /**Syncs the relative encoders of the swerve modules with the CANCoders.
   * 
   */
  public void syncEncoders() {
    frontRightModule.syncEncoders();
    frontLeftModule.syncEncoders();
    backRightModule.syncEncoders();
    backLeftModule.syncEncoders();
  }

  /**
   * @return Y coordinate of the turret on the field
   */
  public double getTurretY(){
    /*
     * If a tag is present in a camera, pull the x and y components of the vision-esimated robot pose. Otherwise,
     * pull the x and y components of the robot pose from the SwerveDrivePoseEstimator
     */
    double h = photonVision.tagIsPresentAcrossAllCameras() ? photonVision.getRobotY().get() : swervePoseEstimator.getEstimatedPosition().getY(); //robot y coordinate
    double k = photonVision.tagIsPresentAcrossAllCameras() ? photonVision.getRobotX().get() : swervePoseEstimator.getEstimatedPosition().getX(); //robot x coordinate
    double robotAngle = ((getGyro().getYaw() % 360.0) + 360) % 360; //Robot rotation

    // Add the turret x and y offsets to the robot x and y position.
    double y = h + TurretConstants.TURRET_OFFSET_Y;
    double x = k + TurretConstants.TURRET_OFFSET_X;

    /*
     * Plug the values into the equation. We found this equation while playing around on the Desmos
     * Graphing Calculator. This equation calculates the position of an object that pivots around
     * a center point.
     */
    return h +((y-h)*Math.cos(robotAngle)) - ((x-k) * Math.sin(robotAngle));
  }

  /**
   * @return X coordinate of the turret on the field
   */
  public double getTurretX(){
    /*
     * If a tag is present in a camera, pull the x and y components of the vision-esimated robot pose. Otherwise,
     * pull the x and y components of the robot pose from the SwerveDrivePoseEstimator
     */
    double h = photonVision.tagIsPresentAcrossAllCameras() ? photonVision.getRobotY().get() : swervePoseEstimator.getEstimatedPosition().getY(); //robot y coordinate
    double k = photonVision.tagIsPresentAcrossAllCameras() ? photonVision.getRobotX().get() : swervePoseEstimator.getEstimatedPosition().getX(); //robot x coordinate
    double robotAngle = ((getGyro().getYaw() % 360.0) + 360) % 360; //Robot rotation in degrees

    // Add the turret x and y offsets to the robot x and y position.
    double y = h + TurretConstants.TURRET_OFFSET_Y; //Original y coordinate of turret relative to robot center y coordinate
    double x = k + TurretConstants.TURRET_OFFSET_X; //Original x coordinate of turret relative to robot center x coordinate

    /*
     * Plug the values into the equation. We found this equation while playing around on the Desmos
     * Graphing Calculator. This equation calculates the position of an object that pivots around
     * a center point.
     */
    return k +((y-h)*Math.sin(robotAngle)) + ((x-k) * Math.cos(robotAngle));
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

  /**Gets all the rotate encoder positions in radians.
   * 
   * @return An array containing all the rotate encoder positions in radians.
   */
  public double[] getRotateEncoderPosition() {
    double[] rotations = new double[4];

    for(int i = 0; i < 4; i++) {
      rotations[i] = swerveModules[i].absoluteRotatePosition();
    }

    return rotations;
  }

  /**Gets all the drive motor velocities in radians/second.
   * 
   * @return An array containing all the drive motor velocities in radians/second.
   */
  public double getDriveEncoderVelocity() {
    double velocity = 0.0;

    for(int i = 0; i < 4; i++) {
      velocity += swerveModules[i].getDriveVelocityRad() / 4.0; 
    }

    return velocity;
  }

  /**Puts the front left module values onto Elastic.
   * 
   * @param sendableBuilder The SendableBuilder to use.
   */
  public void putFrontLeftValues(SendableBuilder sendableBuilder){
    sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Rotate Encoder(Radians): " , ()-> frontLeftModule.getRotateEncoderPosition(), null);
    sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Rotate Setpoint", () -> frontLeftModule.getRotateSetpoint(), null);
    sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Absolute Position " , ()-> frontLeftModule.absoluteRotatePosition(), null);
    sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Velocity", () -> frontLeftModule.getDriveVelocity(), null);
    sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " drive setpoint", () -> (frontLeftModule.getDriveSetpoint()), null);
    sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Drive Voltage", () -> frontLeftModule.getDriveVoltage(), null);
    sendableBuilder.addDoubleProperty(frontLeftModule.printLabel() + " Drive Position", () -> frontLeftModule.getDrivePosition(), null);
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
    sendableBuilder.addDoubleProperty(frontRightModule.printLabel() + " Drive Voltage", () -> frontRightModule.getDriveVoltage(), null);
    sendableBuilder.addDoubleProperty(frontRightModule.printLabel() + " Drive Position", () -> frontRightModule.getDrivePosition(), null);
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
    sendableBuilder.addDoubleProperty(backLeftModule.printLabel() + " Drive Voltage", () -> backLeftModule.getDriveVoltage(), null);
    sendableBuilder.addDoubleProperty(backLeftModule.printLabel() + " Drive Position", () -> backLeftModule.getDrivePosition(), null);
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
    sendableBuilder.addDoubleProperty(backRightModule.printLabel() + " Drive Voltage", () -> backRightModule.getDriveVoltage(), null);
    sendableBuilder.addDoubleProperty(backRightModule.printLabel() + " Drive Position", () -> backRightModule.getDrivePosition(), null);
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
    sendableBuilder.addBooleanProperty("Shoot While Moving Mode", () -> shootWhileMoving, null);

    sendableBuilder.addDoubleProperty("Gyro Reading", ()-> gyroRad(), null); //NAVX USED TO BE HERE
    sendableBuilder.addDoubleProperty("Raw Gyro Reading", ()-> NavX.getAngle(), null); //NAVX USED TO BE HERE

    sendableBuilder.addDoubleProperty("FL Distance Travelled", ()-> frontLeftModule.getDistance(), null);
    sendableBuilder.addDoubleProperty("FL Velocity", ()-> frontLeftModule.getDriveVelocity(), null);

    sendableBuilder.addDoubleProperty("Pose2d  X", () ->  swervePoseEstimator.getEstimatedPosition().getX(), null);
    sendableBuilder.addDoubleProperty("Pose2d  Y", () ->  swervePoseEstimator.getEstimatedPosition().getY(), null);

    sendableBuilder.addDoubleProperty("Rotations", () ->  swervePoseEstimator.getEstimatedPosition().getRotation().getRadians(), null);

    sendableBuilder.addDoubleProperty("Chassis speeds, X", () -> getChassisSpeeds().vxMetersPerSecond, null);
    sendableBuilder.addDoubleProperty("Chassis speeds, Y", () -> getChassisSpeeds().vyMetersPerSecond, null);
    sendableBuilder.addDoubleProperty("Chassis speeds, rotation", () -> getChassisSpeeds().omegaRadiansPerSecond, null);

    sendableBuilder.addBooleanProperty("Am I red?", () -> redSide, null);

    sendableBuilder.addDoubleProperty("Match Time", () -> DriverStation.getMatchTime(), null);

    sendableBuilder.addDoubleProperty("Average Drive Velocity", () -> getDriveEncoderVelocity(), null);
    sendableBuilder.addDoubleArrayProperty("All Wheel Rotations", () -> getRotateEncoderPosition(), null);
    
    SmartDashboard.putData(field);
  }
      
  @Override
  public void periodic() {
    /*
     * Only use the .addVisionMeasurement() method if there is a tag present. If you don't have a tag present
     * in any of the cameras, then don't attempt to add any new vision measurements to the SwerveDrivePoseEstimator.
     */
    if(photonVision.tagIsPresentAcrossAllCameras()) {
      // photonVision.updatePose3d(new Rotation3d(new Rotation2d(gyroRad())));
      // photonVision.updatePose2d();

      swervePoseEstimator.addVisionMeasurement(
        photonVision.getRobotPose2d().get(),
        photonVision.getTimestampOfPose()
        // photonVision.getVisionStandardDeviations()
      );
    }

    // photonVision.addHeadingData(Timer.getFPGATimestamp(), new Rotation3d(new Rotation2d(gyroRad())));

    /*
     * Updates the odometry pose estimate while compensating for latency. If a vision measurement is added,
     * then the vision measurement will be taken into account.
     */
    swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(gyroRad()), getCurrentSwerveModulePositions());

    /*
     * Sets the estimated robot pose on the image of the field.
     */
    field.setRobotPose(swervePoseEstimator.getEstimatedPosition());

    /*
     * Sets the pose2d of the turret
     * 
     * The rotation (relative to the robot) of the turret is obtained from:
     * 1. taking the total accumulated yaw angle of the gyro in degrees
     * 2. using the mod operator to ensure the angle is between 0-360 degrees
     * 3. subtracting 180 from the number, because the turret's default position is 180 degrees from the robot's default rotation
     * 4. applying a negative to the entire equation to flip the sign, because we subtracted by 180
     */
    photonVision.setTurretPose2d(getTurretX(), getTurretY(), -(getGyro().getAngle() % 360) - 180);
  }
}