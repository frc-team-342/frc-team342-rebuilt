// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
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

  private SysIdRoutine topShooterSysIDRoutine;
  private SysIdRoutine bottomShooterSysIDRoutine;

  private boolean isShooting;
  /** Creates a new Shooter. */
  public Shooter(PhotonVision photonVision) {
    topShooterMotor = new SparkFlex(TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    bottomShooterMotor =  new SparkFlex(BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    bottomFeederMotor = new SparkFlex(BOTTOM_FEEDER_MOTOR_ID, MotorType.kBrushless);
    topFeederMotor = new SparkFlex(TOP_FEEDER_MOTOR_ID, MotorType.kBrushless);
    spindexerMotor = new SparkFlex(SPINDEXER_ID, null);

    topShooterEncoder = topShooterMotor.getEncoder();
    bottomShooterEncoder = bottomShooterMotor.getEncoder();
    bottomFeederEncoder = bottomFeederMotor.getEncoder();
    topFeederEncoder = topFeederMotor.getEncoder();

    topShooterMotorConfig = new SparkFlexConfig();
    bottomShooterMotorConfig = new SparkFlexConfig();
    bottomFeederMotorConfig = new SparkFlexConfig();
    topFeederMotorConfig = new SparkFlexConfig();
    spindexerMotorConfig = new SparkFlexConfig();

    topShooterPID = topShooterMotor.getClosedLoopController();
    bottomShooterPID = bottomShooterMotor.getClosedLoopController();

    topShooterMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60)
      .inverted(true);

    topShooterMotorConfig.encoder
      .velocityConversionFactor(TOP_SHOOTER_VELOCITY_CONVERSION_FACTOR);
    
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

    topShooterMap = new InterpolatingDoubleTreeMap();
    bottomShooterMap = new InterpolatingDoubleTreeMap();
    flightTimeMap = new InterpolatingDoubleTreeMap();

    mapShooterVelocities();
    mapShooterFlightTimes();

    isShooting = false;

    this.photonVision = photonVision;

    topShooterSysIDRoutine = new SysIdRoutine(
      new Config(
        Volts.of(2).per(Second),
        Volts.of(8),
        Seconds.of(30)
      ),
      new Mechanism(
        (volts) -> topShooterMotor.setVoltage(volts.in(Volts)), null, this)
    );

    bottomShooterSysIDRoutine = new SysIdRoutine(
      new Config(
        Volts.of(2).per(Second),
        Volts.of(8),
        Seconds.of(30)
      ),
      new Mechanism(
        (volts) -> bottomShooterMotor.setVoltage(volts.in(Volts)), null, this)
    );
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
    return Commands.sequence(
      topShooterSysIDRoutine
        .quasistatic(Direction.kForward)
        .withTimeout(5),
      new WaitCommand(3),
      topShooterSysIDRoutine
        .quasistatic(Direction.kReverse)
        .withTimeout(5),
      new WaitCommand(3),
      topShooterSysIDRoutine
        .dynamic(Direction.kForward)
        .withTimeout(5),
      new WaitCommand(3),
      topShooterSysIDRoutine
        .dynamic(Direction.kReverse)
        .withTimeout(5)
    );
  }

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

  /**Sets the target velocity (in m/s) of both shooter motors based on distance from the hub.
   * Target velocities are obtained via interpolation.
   * Spins the spindexer and feeder as well.
   * 
   * @param speed The speed to set the feeder to.
   * @param pose The pose of the turret.
   */
  public Command shootWithDistance(double speed, Pose2d pose) {
    return Commands.parallel(
      run(() -> topShooterPID.setSetpoint(getTopTargetVelocity(photonVision.getDistanceToHub(pose)), ControlType.kVelocity)),
      run(() -> bottomShooterPID.setSetpoint(getBottomTargetVelocity(photonVision.getDistanceToHub(pose)), ControlType.kVelocity)),
      run(() -> feed(speed)),
      new WaitCommand(1).andThen(() -> spinSpindexer())
    );
  }

  /**Sets the target velocity of both shooter motors to the given velocity.
   * Spins the feeder and spindexer too.
   * 
   * @param topShooterSpeed The velocity (in m/s) to set the top shooter motor to.
   * @param bottomShooterSpeed The velocity (in m/s) to set the bottom shooter motor to.
   * @param feederSpeed The velocity (in m/s) to set the feeder to.
   */
  public Command shootWithSpeed(double topShooterSpeed, double bottomShooterSpeed, double feederSpeed) {
    return Commands.parallel(
      run(() -> topShooterPID.setSetpoint(topShooterSpeed, ControlType.kVelocity)),
      run(() -> bottomShooterPID.setSetpoint(bottomShooterSpeed, ControlType.kVelocity)),
      run(() -> feed(feederSpeed)),
      new WaitCommand(1).andThen(() -> spinSpindexer())
    );
  }

  /**Sets both shooter motors to the given speed.
   * Spins the feeder and spindexer as well.
   * This method does not use a closed loop controller.
   * 
   * @param topShooterSpeed The speed to set the top shooter motor to.
   * @param bottomShooterSpeed The speed to set the bottom shooter motor to.
   * @param feederSpeed The speed to set the feeder to.
   */
  public Command shootWithoutPID(double topShooterSpeed, double bottomShooterSpeed, double feederSpeed) {
    return Commands.parallel(
      run(() -> topShooterMotor.set(topShooterSpeed)),
      run(() -> bottomShooterMotor.set(bottomShooterSpeed)),
      run(() -> feed(feederSpeed)),
      new WaitCommand(1).andThen(() -> spinSpindexer())
    );
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
    if(isShooting) {
      spindexerMotor.set(0.6);
    }
    else{
      spindexerMotor.set(0);
    }
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
    builder.addBooleanProperty("Able to Shoot", () -> photonVision.tagIsPresentAcrossAllCameras(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
