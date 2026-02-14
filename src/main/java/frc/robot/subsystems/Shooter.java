// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.Spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Shooter extends SubsystemBase {
  private SparkMax topShooterMotor;
  private SparkMax bottomShooterMotor;
  private SparkMax feederMotor;

  private RelativeEncoder topShooterEncoder;
  private RelativeEncoder bottomShooterEncoder;
  private RelativeEncoder feederEncoder;

  private SparkMaxConfig topShooterMotorConfig;
  private SparkMaxConfig bottomShooterMotorConfig;
  private SparkMaxConfig feederMotorConfig;

  private SparkClosedLoopController topShooterPID;
  private SparkClosedLoopController bottomShooterPID;

  private InterpolatingDoubleTreeMap topShooterMap;
  private InterpolatingDoubleTreeMap bottomShooterMap;

  private Spindexer spindexer;
  /** Creates a new Shooter. */
  public Shooter(Spindexer spindexer) {
    topShooterMotor = new SparkMax(TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    bottomShooterMotor =  new SparkMax(BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    topShooterEncoder = topShooterMotor.getEncoder();
    bottomShooterEncoder = bottomShooterMotor.getEncoder();
    feederEncoder = feederMotor.getEncoder();

    topShooterMotorConfig = new SparkMaxConfig();
    bottomShooterMotorConfig = new SparkMaxConfig();
    feederMotorConfig = new SparkMaxConfig();

    topShooterPID = topShooterMotor.getClosedLoopController();
    bottomShooterPID = bottomShooterMotor.getClosedLoopController();

    topShooterMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60);
    
    topShooterMotorConfig.closedLoop
      .pid(TOP_SHOOTER_PID_VALUES[0], TOP_SHOOTER_PID_VALUES[1], TOP_SHOOTER_PID_VALUES[2], ClosedLoopSlot.kSlot0);
    
    bottomShooterMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60);

    bottomShooterMotorConfig.closedLoop
      .pid(BOTTOM_SHOOTER_PID_VALUES[0], BOTTOM_SHOOTER_PID_VALUES[1], BOTTOM_SHOOTER_PID_VALUES[2], ClosedLoopSlot.kSlot0);

    feederMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60);

    topShooterMotor.configure(topShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomShooterMotor.configure(bottomShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topShooterMap = new InterpolatingDoubleTreeMap();
    bottomShooterMap = new InterpolatingDoubleTreeMap();

    //TODO: insert more values after testing
    put(0.0, 0.0, 0.0);

    this.spindexer = spindexer;
    spindexer.setShooting(false);
  }

  /**Gets the velocity of the shooter motor controlling the top wheels.
   * 
   * @return The velocity (in RPM) of the top shooter motor.
   */
  public double getTopShooterVelocity() {
    return topShooterEncoder.getVelocity();
  }

  /**Gets the velocity of the shooter motor controlling the bottom wheels.
   * 
   * @return The velocity (in RPM) of the bottom shooter motor.
   */
  public double getBottomShooterVelocity() {
    return bottomShooterEncoder.getVelocity();
  }

  /**Gets the velocity of the feeder motor.
   * 
   * @return The velocity (in RPM) of the feeder motor;
   */
  public double getFeederVelocity() {
    return feederEncoder.getVelocity();
  }

  /**Gets the target velocity of the top shooter motor based on distance from the hub.
   * 
   * @param distance The distance (in meters) from the hub.
   * @return The target velocity of the top shooter motor (in RPM).
   */
  public double getTopTargetRPM(double distance) {
    return topShooterMap.get(distance);
  }

  /**Gets the target velocity of the bottom shooter motor based on distance from the hub.
   * 
   * @param distance The distance (in meters) from the hub.
   * @return The target velocity of the bottom shooter motor (in RPM).
   */
  public double getBottomTargetRPM(double distance) {
    return bottomShooterMap.get(distance);
  }

  /**Adds a new entry into the top and bottom shooter interpolation maps.
   * 
   * @param meters The distance (in meters) from the hub.
   * @param topVelocity The velocity (in RPM) of the top shooter motor.
   * @param bottomVelocity The velocity (in RPM) of the bottom shooter motor.
   */
  public void put(double meters, double topVelocity, double bottomVelocity) {
    topShooterMap.put(meters, topVelocity);
    bottomShooterMap.put(meters, bottomVelocity);
  }

  /**Sets the target velocity (in RPM) of both shooter motors based on distance from the hub.
   * Target velocities are pulled from the interpolation maps.
   * 
   * @param distance The distance (in meters) from the hub.
   */
  public void shootWithDistance(double distance) {
    topShooterPID.setSetpoint(getTopTargetRPM(distance), ControlType.kVelocity);
    bottomShooterPID.setSetpoint(getBottomTargetRPM(distance), ControlType.kVelocity);
    spindexer.setShooting(true);
  }

  /**Sets the target velocity of both shooter motors to the given velocity.
   * 
   * @param speed The velocity (in RPM) to set both motors to.
   */
  public void shootWithSpeed(double speed) {
    topShooterPID.setSetpoint(speed, ControlType.kVelocity);
    bottomShooterPID.setSetpoint(speed, ControlType.kVelocity);
    spindexer.setShooting(true);
  }

  /**Sets both shooter motors to the given speed.
   * This method does not use a closed loop controller.
   * 
   * @param speed The speed (in RPM) to set both motors to.
   */
  public void shootWithoutPID(double speed) {
    topShooterMotor.set(speed);
    bottomShooterMotor.set(speed);
  }

  /**Stops both shooter motors.
   * 
   */
  public void stopShooter() {
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
  }

  /**Stops the feeder motor.
   * 
   */
  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  //Putting shooter data onto Elastic
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("Top Shooter Velocity", () -> getTopShooterVelocity(), null);
    builder.addDoubleProperty("Bottom Shooter Velocity", () -> getBottomShooterVelocity(), null);
    builder.addDoubleProperty("Feeder Shooter", () -> getFeederVelocity(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
