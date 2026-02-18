// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

public class Intake extends SubsystemBase {
  private SparkFlex intakeMotor;
  private SparkFlex wristMotor;

  private RelativeEncoder intakeEncoder;
  private DutyCycleEncoder throughBore;

  private SparkClosedLoopController wristPID;

  private SparkFlexConfig intakeConfig;
  private SparkFlexConfig wristConfig;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new SparkFlex(INTAKE_ID, MotorType.kBrushless);
    wristMotor = new SparkFlex(WRIST_ID, MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    throughBore = new DutyCycleEncoder(WRIST_ENCODER_ID);

    wristPID = wristMotor.getClosedLoopController();

    intakeConfig = new SparkFlexConfig();
    wristConfig = new SparkFlexConfig();

    intakeConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    wristConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    wristConfig.absoluteEncoder
      .positionConversionFactor(WRIST_POSITION_CONVERSION_FACTOR)
      .zeroOffset(WRIST_ENCODER_ZERO_OFFSET);

    wristConfig.closedLoop
      .allowedClosedLoopError(WRIST_ALLOWED_ERROR, ClosedLoopSlot.kSlot0)
      .outputRange(-1, 1)
      .positionWrappingEnabled(false)
      .pid(WRIST_PID_VALUES[0], WRIST_PID_VALUES[1], WRIST_PID_VALUES[2], ClosedLoopSlot.kSlot0);

    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**Spins the intake motor at the given speed.
   * 
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  public void spinIntake(double speed) {
    intakeMotor.set(speed);
  }

  /**Moves the wrist motor at the given speed.
   * 
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  public void moveWrist(double speed) {
    wristMotor.set(speed);
  }

  /**Moves the wrist to the given position.
   * 
   * @param setpoint The position to move to.
   */
  public void wristToPosition(double setpoint) {
    wristPID.setSetpoint(setpoint, ControlType.kPosition);
  }

  /**Stops the wrist motor.
   * 
   */
  public void stopWrist() {
    wristMotor.stopMotor();
  }

  /**Stops the intake motor.
   * 
   */
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  /**Gets the velocity of the intake in RPM.
   * 
   * @return The velocity of the intake in RPM.
   */
  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  /**Gets the absolute position of the wrist.
   * 
   * @return The absolute position of the wrist.
   */
  public double getWristPosition() {
    return throughBore.get();
  }

  /**Gets the wrist encoder.
   * 
   * @return The wrist encoder.
   */
  public DutyCycleEncoder getWristEncoder() {
    return throughBore;
  }

  //Putting intake and wrist data onto Elastic
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Intake");

    builder.addDoubleProperty("Intake Velocity", () -> getIntakeVelocity(), null);
    builder.addDoubleProperty("Wrist Position", () -> getWristPosition(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
