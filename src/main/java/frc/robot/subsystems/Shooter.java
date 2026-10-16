// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import frc.robot.CustomXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Shooter extends SubsystemBase {
  private SparkFlex topShooterMotor;
  private SparkFlex bottomShooterMotor;
  private SparkFlex bottomFeederMotor;
  private SparkFlex topFeederMotor;
  private SparkFlex spindexerMotor;

  private RelativeEncoder topShooterEncoder;
  private RelativeEncoder bottomShooterEncoder;
  private RelativeEncoder bottomFeederEncoder;
  private RelativeEncoder topFeederEncoder;

  private SparkFlexConfig topShooterMotorConfig;
  private SparkFlexConfig bottomShooterMotorConfig;
  private SparkFlexConfig bottomFeederMotorConfig;
  private SparkFlexConfig topFeederMotorConfig;
  private SparkFlexConfig spindexerMotorConfig;

  private SparkClosedLoopController topShooterPID;
  private SparkClosedLoopController bottomShooterPID;

  private InterpolatingDoubleTreeMap topShooterMap;
  private InterpolatingDoubleTreeMap bottomShooterMap;
  private InterpolatingDoubleTreeMap flightTimeMap;

  private PhotonVision photonVision;
  private CustomXboxController controller;

  private SysIdRoutine topShooterSysIDRoutine;
  private SysIdRoutine bottomShooterSysIDRoutine;

  private double joystickPercentOutput;

  private boolean joystickControl;
  
  /** Creates a new Shooter. */
  public Shooter(PhotonVision photonVision, CustomXboxController controller) {
    topShooterMotor = new SparkFlex(TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    bottomShooterMotor =  new SparkFlex(BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    bottomFeederMotor = new SparkFlex(BOTTOM_FEEDER_MOTOR_ID, MotorType.kBrushless);
    topFeederMotor = new SparkFlex(TOP_FEEDER_MOTOR_ID, MotorType.kBrushless);
    spindexerMotor = new SparkFlex(SPINDEXER_ID, MotorType.kBrushless);

    topShooterEncoder = topShooterMotor.getEncoder();
    bottomShooterEncoder = bottomShooterMotor.getEncoder();
    bottomFeederEncoder = bottomFeederMotor.getEncoder();
    topFeederEncoder = topFeederMotor.getEncoder();

    topShooterMotorConfig = new SparkFlexConfig();
    bottomShooterMotorConfig = new SparkFlexConfig();
    bottomFeederMotorConfig = new SparkFlexConfig();
    topFeederMotorConfig = new SparkFlexConfig();
    spindexerMotorConfig = new SparkFlexConfig();

    /*
     * We use two different PID controllers for the top and bottom shooter motors.
     * This is for one main reason: the wheels have different weights and diameters,
     * so there will be a difference in their PID constants, as the wheels' ability
     * to hold speed will be different.
     */
    topShooterPID = topShooterMotor.getClosedLoopController();
    bottomShooterPID = bottomShooterMotor.getClosedLoopController();

    topShooterMotorConfig
      /*
       * For our entire shooter mechanism, all the motors are set to coast mode.
       * This is because we're spinning all the motors at extremely high speeds,
       * and we don't want to potentially damage anything by forcing the motors
       * to immediately stop when the shoot command ends.
       */
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60)
      .inverted(true);

    //Conversion from RPM to m/s
    topShooterMotorConfig.encoder
      .velocityConversionFactor(TOP_SHOOTER_VELOCITY_CONVERSION_FACTOR);
    
    /*
     * Along with the regular PID configurations, we also have to add SVA values to the feed forward for the
     * PID controllers. SVA stands for kS, kV, and kA. kS refers to the voltage required for the
     * mechanism to overcome static friction (or, voltage required for motion to begin), kV refers
     * to the voltage required for the mechanism to hold a constant velocity, and kA refers to
     * the voltage required for the mechanism to hold a constant acceleration.
     * 
     * Basic position PID does not require the usage of SVA, as you usually hold a position rather
     * than a velocity. However, since velocity PID typically holds the mechanism at a certain
     * velocity, SVA is required. Depending on the mechanism, you may only need an estimated
     * kV value. However, we've found that we've typically needed at least kS and kV values, which we
     * obtain from system identification.
     */
    topShooterMotorConfig.closedLoop
      .allowedClosedLoopError(SHOOTER_VELOCITY_ERROR, ClosedLoopSlot.kSlot0)
      .pid(TOP_SHOOTER_PID_VALUES[0], TOP_SHOOTER_PID_VALUES[1], TOP_SHOOTER_PID_VALUES[2], ClosedLoopSlot.kSlot0)
      .feedForward.sva(TOP_SHOOTER_SVA_VALUES[0], TOP_SHOOTER_SVA_VALUES[1], TOP_SHOOTER_SVA_VALUES[2]);
    
    bottomShooterMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60)
      .inverted(true);

    bottomShooterMotorConfig.encoder
      .velocityConversionFactor(BOTTOM_SHOOTER_VELOCITY_CONVERSION_FACTOR);

    bottomShooterMotorConfig.closedLoop
      .allowedClosedLoopError(SHOOTER_VELOCITY_ERROR, ClosedLoopSlot.kSlot0)
      .pid(BOTTOM_SHOOTER_PID_VALUES[0], BOTTOM_SHOOTER_PID_VALUES[1], BOTTOM_SHOOTER_PID_VALUES[2], ClosedLoopSlot.kSlot0)
      .feedForward.sva(BOTTOM_SHOOTER_SVA_VALUES[0], BOTTOM_SHOOTER_SVA_VALUES[1], BOTTOM_SHOOTER_SVA_VALUES[2]);

    bottomFeederMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60);

    topFeederMotorConfig
      .apply(bottomFeederMotorConfig)
      .follow(BOTTOM_FEEDER_MOTOR_ID);

    spindexerMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60);

    topShooterMotor.configure(topShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomShooterMotor.configure(bottomShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomFeederMotor.configure(bottomFeederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topFeederMotor.configure(topFeederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spindexerMotor.configure(spindexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    /*
     * For interpolation, you're able to create a InterpolatingDoubleTreeMap
     * and store keys/values in it, so you don't have to create your own array.
     * 
     * Interpolation is a method used to estimate an unknown value that falls
     * between known data points. In simpler terms, a line of best fit is created
     * using provided values. Whenever you want to pull a value, the line of best fit
     * is used to estimate that value.
     * 
     * In our case, we used to (we've switched to using cubic regression) use interpolation
     * to determine the appropriate velocity of the shooter at a certain distance.
     * We input multiple distance values and their corresponding velocities. Then
     * when we input the distance of the robot from the hub, the interpolation
     * map returns the estimated velocity.
     */
    topShooterMap = new InterpolatingDoubleTreeMap();
    bottomShooterMap = new InterpolatingDoubleTreeMap();
    flightTimeMap = new InterpolatingDoubleTreeMap();

    //Adding values to interpolation maps
    mapShooterVelocities();
    mapShooterFlightTimes();

    this.photonVision = photonVision;
    this.controller = controller;

    /*
     * SysId (system identification) is used to find certain values for a mechanism.
     * We use it to determine the kS, kV, and kA values for mechanisms.
     * 
     * First, declare and then instantiate the SysIdRoutine.
     * You will need to input a Config and Mechanism object. This can be
     * done by simply creating new Config and Mechanism objects.
     * 
     * The Config object requires the voltage ramp rate for the quasistatic test,
     * the step voltage for the dynamic test, and the safety timeout. The quasistatic
     * test will add the number inputted for voltage ramp rate every second the test runs for.
     * The dynamic test will immediately jump to the voltage inputted. After the test duration
     * becomes longer than the safety timeout, the system identification routine ends.
     * 
     * The Mechanism object requires a method to set the voltage of the mechanism, a logger,
     * and a subsystem. Simply use [motorname].setVoltage() for the method, null for the logger
     * (because WPILib logs all the information automatically), and the "this" keyword for the
     * required subsystem.
     */
    topShooterSysIDRoutine = new SysIdRoutine(
      new Config(
        Volts.of(2).per(Second),
        Volts.of(8),
        Seconds.of(30)
      ),
      new Mechanism(
        (volts) -> topShooterMotor.setVoltage(volts.in(Volts)), null, this)
    );

    /*
     * We do separate routines, because the top and bottom shooters will
     * have different SVA values.
     */
    bottomShooterSysIDRoutine = new SysIdRoutine(
      new Config(
        Volts.of(2).per(Second),
        Volts.of(8),
        Seconds.of(30)
      ),
      new Mechanism(
        (volts) -> bottomShooterMotor.setVoltage(volts.in(Volts)), null, this)
    );

    /*
     * Outreach only
     * 
     * Allows for manual control of shooter percentage output using a joystick.
     */
    joystickPercentOutput = 0.0;
    joystickControl = false;
  }

  /**Gets the velocity of the shooter motor controlling the top wheels.
   * 
   * @return The velocity (in m/s) of the top shooter motor.
   */
  public double getTopShooterVelocity() {
    return topShooterEncoder.getVelocity();
  }

  /**Gets the velocity of the shooter motor controlling the bottom wheels.
   * 
   * @return The velocity (in m/s) of the bottom shooter motor.
   */
  public double getBottomShooterVelocity() {
    return bottomShooterEncoder.getVelocity();
  }

  /**Gets the velocity of the bottom feeder motor.
   * 
   * @return The velocity (in m/s) of the feeder motor;
   */
  public double getBottomFeederVelocity() {
    return bottomFeederEncoder.getVelocity();
  }

  /**Gets the velocity of the top feeder motor.
   * 
   * @return The velocity (in m/s) of the feeder motor.
   */
  public double getTopFeederVelocity() {
    return topFeederEncoder.getVelocity();
  }

  /**Gets the target velocity of the top shooter motor based on distance from the hub.
   * 
   * @param distance The distance (in meters) from the hub.
   * @return The target velocity of the top shooter motor (in m/s).
   */
  public double getTopTargetVelocity(double distance) {
    return topShooterMap.get(distance);
  }

  /**Gets the target velocity of the bottom shooter motor based on distance from the hub.
   * 
   * @param distance The distance (in meters) from the hub.
   * @return The target velocity of the bottom shooter motor (in m/s).
   */
  public double getBottomTargetVelocity(double distance) {
    return bottomShooterMap.get(distance);
  }

  /**Gets the flight time of the fuel based on distance from the hub.
   * 
   * @param distance The distance (in meters) from the hub.
   * @return The flight (in seconds) of the fuel.
   */
  public double getFlightTime(double distance) {
    return flightTimeMap.get(distance);
  }

  /**Gets the voltage of the top shooter.
   * 
   * @return The voltage of the top shooter.
   */
  public double getTopShooterVoltage() {
    return topShooterMotor.getAppliedOutput() * topShooterMotor.getBusVoltage();
  }

  /**Gets the voltage of the bottom shooter.
   * 
   * @return The voltage of the bottom shooter.
   */
  public double getBottomShooterVoltage() {
    return bottomShooterMotor.getAppliedOutput() * bottomShooterMotor.getBusVoltage();
  }

  /**Gets the position of the top shooter encoder.
   * 
   * @return The position of the top shooter encoder.
   */
  public double getTopShooterPosition() {
    return topShooterEncoder.getPosition();
  }

  /**Gets the position of the bottom shooter encoder.
   * 
   * @return The position of the bottom shooter encoder.
   */
  public double getBottomShooterPosition() {
    return bottomShooterEncoder.getPosition();
  }

  /**Runs the SysIdRoutine for the top shooter.
   * 
   * @return The top shooter's SysIdRoutine.
   */
  public Command runTopShooterSysID() {
    /*
     * We have to return a command for this method, because
     * we're going to run the routine as an auto.
     */
    return Commands.sequence(
      /*
       * Start with quasistatic forward
       * 
       * Spin the motor for 5 seconds; any amount of time works,
       * but you don't want to spin it for too short of a time
       * or too long of a time.
       */
      topShooterSysIDRoutine
        .quasistatic(Direction.kForward)
        .withTimeout(5),
      //Wait 3 seconds before spinning in reverse to allow the wheel to slow down
      new WaitCommand(3),
      topShooterSysIDRoutine
      //Quasistatic reverse for 5 seconds
        .quasistatic(Direction.kReverse)
        .withTimeout(5),
      new WaitCommand(3),
      //Move onto dynamic forward for 5 seconds
      topShooterSysIDRoutine
        .dynamic(Direction.kForward)
        .withTimeout(5),
      new WaitCommand(3),
      topShooterSysIDRoutine
      //Dynamic reverse for 5 seconds
        .dynamic(Direction.kReverse)
        .withTimeout(5)
    );
  }

  /*
   * Do the same thing for the bottom shooter
   * 
   * Also, looks like someone accidentally put "bottom" twice.
   * How did no one notice that until now?
   */

  /**Runs the SysIdRoutine for the bottom shooter.
   * 
   * @return The bottom shooter's SysIdRoutine.
   */
  public Command runBottombottomShooterSysID() {
    return Commands.sequence(
      bottomShooterSysIDRoutine
        .quasistatic(Direction.kForward)
        .withTimeout(5),
      new WaitCommand(3),
      bottomShooterSysIDRoutine
        .quasistatic(Direction.kReverse)
        .withTimeout(5),
      new WaitCommand(3),
      bottomShooterSysIDRoutine
        .dynamic(Direction.kForward)
        .withTimeout(5),
      new WaitCommand(3),
      bottomShooterSysIDRoutine
        .dynamic(Direction.kReverse)
        .withTimeout(5)
    );
  }

  /*
   * We used to use interpolation for estimating velocity values at certain distances.
   * Now, we use cubic regression, because we found that it provided more accurate
   * velocities at shorter and longer distances. Basically, we went onto the
   * Desmos graphing calculator, created a table, input the distances as
   * x and velocities as y (top shooter as y1, bottom as y2),
   * then used the built-in Desmos regression to create 2 lines of best fit
   * using cubic equations (1 for each shooter motor). We then copied those
   * cubic equations into the code, which are seen in the 2 methods below.
   */

  /**Uses cubic regression to calculate the top shooter wheel velocity.
   * 
   * @param meters Distance from the hub (meters).
   * @return Top shooter wheel velocity.
   */
  public double getTopRegressionVelocity(double meters) {
    // return (0.156757 * Math.pow(meters, 3)) - (1.42652 * Math.pow(meters, 2)) + (5.2989 * meters) + 2.48681;
    return (0.0597356 * Math.pow(meters, 3)) - (0.588908 * Math.pow(meters, 2)) + (3.04758 * meters) + 4.26182;
  }

  /**Uses cubic regression to calculate the bottom shooter wheel velocity.
   * 
   * @param meters Distance from the hub (meters).
   * @return Bottom shooter wheel velocity.
   */
  public double getBottomRegressionVelocity(double meters) {
    // return (0.0733766 * Math.pow(meters, 3)) - (0.71344 * Math.pow(meters, 2)) + (3.03522 * meters) + 5.13096;
    return (0.00898138 * Math.pow(meters, 3)) - (0.159473 * Math.pow(meters, 2)) + (1.57038 * meters) + 6.22942;
  }

  /**Gets the right joystick y axis input.
   * 
   * @return Right joystick y axis input.
   */
  public double getJoystickPercentOutputIncreaseAmount() {
    return controller.getRightY();
  }

  /**Gets the value of the joystick control boolean.
   * 
   * @return The value of the joystick control boolean.
   */
  public boolean getJoystickControlBoolean() {
    return joystickControl;
  }

  /**Adds a new entry into the top and bottom shooter interpolation maps.
   * 
   * @param meters The distance (in meters) from the hub.
   * @param topVelocity The velocity (in m/s) of the top shooter motor.
   * @param bottomVelocity The velocity (in m/s) of the bottom shooter motor.
   */
  public void put(double meters, double topVelocity, double bottomVelocity) {
    topShooterMap.put(meters, topVelocity);
    bottomShooterMap.put(meters, bottomVelocity);
  }

  /**Updates the percent output of the shooter when using joystick control.
   * 
   */
  public void updatePercentJoystickOutput() {
    /*
     * This is "-=" instead of "+=", because moving the joystick up
     * returns a negative value, while moving the joystick down
     * returns a positive value.
     * 
     * By subtracting instead of adding, it makes it so
     * moving the joystick up increases percent output and moving the
     * joystick down decreases percent output. This makes it more
     * intuitive for the operator.
     */
    joystickPercentOutput -= (getJoystickPercentOutputIncreaseAmount() * 0.01);

    /*
     * For whatever reason, the MathUtil.clamp() method was not working,
     * so we made our own. Basically, if the percent output is
     * below 0.2, set it to 0.2. If it's above 0.4, set it to 0.4.
     * This way, the shooter will only be able to shoot at 20-40%.
     */
    if(joystickPercentOutput < 0.2) {
      joystickPercentOutput = 0.2;
    }else if (joystickPercentOutput > 0.4) {
      joystickPercentOutput = 0.4;
    }
  }

  /**Toggles joystick control.
   * 
   */
  public void toggleJoystickControl() {
    joystickControl = !joystickControl;
  }

  /** Puts the shooter velocity points into the velocity interpolation map*/
  public void mapShooterVelocities(){
    put(1.5795869380667578, 7.815,8.285);
    put(1.9252849702151729, 8.34, 8.81);
    put(2.159714161664487, 8.365, 8.835);
    put(2.5729031307368464, 9.06, 9.37);
    put(2.7974383174127144, 9.25, 9.53);
    put(3.102963547816569, 9.85, 9.9);
    put(3.684626975998755, 9.95, 9.9);
    put(4.141048999496158, 10.01, 10.08);
    put(5.560794830193121, 11.4, 10.7);
    put(5.835646600423712, 11.6, 10.9);
  }

  /**Puts shooter flight time points into the flight time interpolation map*/
  public void mapShooterFlightTimes(){
    flightTimeMap.put(2.125070162186267, 0.8383);
    flightTimeMap.put(2.285342474424601, 0.848);
    flightTimeMap.put(2.755924648570551, 0.9343);
    flightTimeMap.put(3.064996018406718, 0.9922);
    flightTimeMap.put(3.5764151871506438, 1.058);
  }

  /**Spins the spindexer with a 1 second startup delay.
   * 
   */
  public Command delayedSpinSpindexer() {
    return Commands.sequence(
      new WaitCommand(1),
      Commands.run(() -> spinSpindexer())
    );
  }

  /**Sets the target velocity (in m/s) of both shooter motors based on distance from the hub.
   * Target velocities are obtained via cubic regression.
   * Spins the spindexer and feeder as well.
   * 
   * @param speed The speed to set the feeder to.
   * @param pose The pose of the turret.
   */
  public void shootWithDistance(double speed, Pose2d pose) {
    topShooterPID.setSetpoint(getTopRegressionVelocity(photonVision.getDistanceToHub(pose)), ControlType.kVelocity);
    bottomShooterPID.setSetpoint(getBottomRegressionVelocity(photonVision.getDistanceToHub(pose)), ControlType.kVelocity);
    feed(speed);
  }

  /**Sets the target velocity of both shooter motors to the given velocity.
   * Spins the feeder and spindexer too.
   * 
   * @param topShooterSpeed The velocity (in m/s) to set the top shooter motor to.
   * @param bottomShooterSpeed The velocity (in m/s) to set the bottom shooter motor to.
   * @param feederSpeed The velocity (in m/s) to set the feeder to.
   */
  public void shootWithSpeed(double topShooterSpeed, double bottomShooterSpeed, double feederSpeed) {
    topShooterPID.setSetpoint(topShooterSpeed, ControlType.kVelocity);
    bottomShooterPID.setSetpoint(bottomShooterSpeed, ControlType.kVelocity);
    feed(feederSpeed);
  }

  /**Sets both shooter motors to the given percent output (0.0 - 1.0).
   * Spins the feeder and spindexer as well.
   * This method does not use a closed loop controller.
   * 
   * @param topShooterSpeed The speed to set the top shooter motor to.
   * @param bottomShooterSpeed The speed to set the bottom shooter motor to.
   * @param feederSpeed The speed to set the feeder to.
   */
  public void shootWithoutPID(double topShooterSpeed, double bottomShooterSpeed, double feederSpeed) {
    topShooterMotor.set(topShooterSpeed);
    bottomShooterMotor.set(bottomShooterSpeed);
    feed(feederSpeed);
  }

  /**Sets the feeder to the given speed.
   * 
   * @param speed The speed to set the feeder to.
   */
  public void feed(double speed) {
    bottomFeederMotor.set(speed);
  }

  /**Stops both shooter motors and the spindexer.
   * 
   */
  public void stopShooter() {
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
    spindexerMotor.stopMotor();
  }

  /**Stops the feeder motors.
   * 
   */
  public void stopFeeder() {
    bottomFeederMotor.stopMotor();
  }

  /**Stops both the shooter and feeder.
   * 
   */
  public void stopShooterAndFeeder() {
    stopShooter();
    stopFeeder();
  }

  /** Spins the spindexer if the shooter is running.*/
  public void spinSpindexer(){
    spindexerMotor.set(0.75);
  }

  /**Spins the spindexer at a set speed.
   * @param speed The speed to spin the spindexer at.
   */
  public void SpindexerWithSpeed(double speed) {
    spindexerMotor.set(speed);
  }

  //Putting shooter data onto Elastic
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("Joystick Percent Output", () -> joystickPercentOutput, null);
    builder.addDoubleProperty("Top Shooter Velocity", () -> getTopShooterVelocity(), null);
    builder.addDoubleProperty("Bottom Shooter Velocity", () -> getBottomShooterVelocity(), null);
    builder.addDoubleProperty("Bottom Feeder Velocity", () -> getBottomFeederVelocity(), null);
    builder.addDoubleProperty("Top Feeder Velocity", () -> getTopFeederVelocity(), null);
    builder.addDoubleProperty("Top Shooter Goal", () -> topShooterPID.getSetpoint(), null);
    builder.addDoubleProperty("Bottom Shooter Goal", () -> bottomShooterPID.getSetpoint(), null);
    builder.addDoubleProperty("Top Shooter Voltage", () -> getTopShooterVoltage(), null);
    builder.addDoubleProperty("Bottom Shooter Voltage", () -> getBottomShooterVoltage(), null);
    builder.addDoubleProperty("Top Shooter Position", () -> getTopShooterPosition(), null);
    builder.addDoubleProperty("Bottom Shooter Position", () -> getBottomShooterPosition(), null);
    builder.addDoubleProperty("Interpolated Top Shooter Velocity", () -> getTopTargetVelocity(photonVision.getDistanceToHub(photonVision.getTurretPose2d().get())), null);
    builder.addDoubleProperty("Interpolated Bottom Shooter Velocity", () -> getBottomTargetVelocity(photonVision.getDistanceToHub(photonVision.getTurretPose2d().get())), null);
    builder.addDoubleProperty("Regression Top Shooter Velocity", () -> getTopRegressionVelocity(photonVision.getDistanceToHub(photonVision.getTurretPose2d().get())), null);
    builder.addDoubleProperty("Regression Bottom Shooter Velocity", () -> getBottomRegressionVelocity(photonVision.getDistanceToHub(photonVision.getTurretPose2d().get())), null);
    builder.addBooleanProperty("Able to Shoot", () -> photonVision.tagIsPresentAcrossAllCameras(), null);
    builder.addBooleanProperty("Joystick Control", () -> getJoystickControlBoolean(), null);
  }

  @Override
  public void periodic() {
    /*
     * Only uncomment the code below if the robot is being used for Outreach.
     * You will have to redeploy code after uncommenting/commenting the code.
     */

    // This method will be called once per scheduler run
    // if(joystickControl == true) {
    //   updatePercentJoystickOutput();
    //   shootWithoutPID(joystickPercentOutput, joystickPercentOutput, 0.5);
    //   spinSpindexer();
    // }else{
    //   Commands.runOnce(() -> stopShooterAndFeeder(), this);
    // }
  }
}
