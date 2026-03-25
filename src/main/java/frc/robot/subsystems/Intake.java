// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;

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
  private RelativeEncoder wristEncoder;
  // private DutyCycleEncoder throughBore;

  private SparkClosedLoopController wristPID;

  private SparkFlexConfig intakeConfig;
  private SparkFlexConfig wristConfig;

  private boolean isManual;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new SparkFlex(INTAKE_ID, MotorType.kBrushless);
    wristMotor = new SparkFlex(WRIST_ID, MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    wristEncoder = wristMotor.getEncoder();

    isManual = false;
    // throughBore = new DutyCycleEncoder(WRIST_ENCODER_ID);

    wristPID = wristMotor.getClosedLoopController();

    intakeConfig = new SparkFlexConfig();
    wristConfig = new SparkFlexConfig();

    intakeConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    wristConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    wristConfig.encoder
      .positionConversionFactor(WRIST_POSITION_CONVERSION_FACTOR);

    wristConfig.closedLoop
      .allowedClosedLoopError(WRIST_ALLOWED_ERROR, ClosedLoopSlot.kSlot0)
      .allowedClosedLoopError(WRIST_ALLOWED_ERROR, ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1)
      .positionWrappingEnabled(false)
      .pid(WRIST_PID_VALUES_SLOT0[0], WRIST_PID_VALUES_SLOT0[1], WRIST_PID_VALUES_SLOT0[2], ClosedLoopSlot.kSlot0)
      .pid(WRIST_PID_VALUES_SLOT1[0], WRIST_PID_VALUES_SLOT1[1], WRIST_PID_VALUES_SLOT1[2], ClosedLoopSlot.kSlot1);

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

  public void toggleManual(){
    isManual = !isManual;
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
  public void wristToPosition(double setpoint, XboxController controller) {
    if(!isManual){
      if(setpoint > getWristPosition()) {
        wristPID.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }else if(setpoint < getWristPosition()) {
        wristPID.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        if(wristPID.isAtSetpoint()) {
          resetWristEncoder();
        }
      }
    }
    else{
      wristWithJoystick(controller);
    }
  }

  public void wristWithJoystick(XboxController controller){
    moveWrist(controller.getLeftY()/5);
  }

  /**Moves the wrist to the given position and sets the intake to the given speed.
   * 
   * @param setpoint The position to move the wrist to.
   * @param speed The speed to set the intake to.
   */
  public void wristAndIntake(double setpoint, double speed, XboxController controller) {
    wristToPosition(setpoint, controller);
    spinIntake(speed);
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

  /**Resets the wrist encoder to 0.0.
   * 
   */
  public void resetWristEncoder() {
    wristEncoder.setPosition(0.0);
  }

  /**Gets the velocity of the intake in RPM.
   * 
   * @return The velocity of the intake in RPM.
   */
  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  /**Gets the position of the wrist.
   * 
   * @return The position of the wrist.
   */
  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  /**Gets the velocity of the wrist.
   * 
   * @return The velocity of the wrist.
   */
  public double getWristVelocity() {
    return wristEncoder.getVelocity();
  }

  /**Gets the voltage of the wrist.
   * 
   * @return The voltage of the wrist.
   */
  public double getWristVoltage() {
    return wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
  }

  /**Gets the wrist encoder.
   * 
   * @return The wrist encoder.
   */
  public RelativeEncoder getWristEncoder() {
    return wristEncoder;
  }

  public boolean isManual() {
    return isManual;
  }

  /**Checks to see if the wrist is at the desired position.
   * 
   * @param position The desired position.
   * @return {@code true} if the wrist is at the desired position, {@code false} otherwise.
   */
  public boolean wristAtPosition(double position) {
    return Math.abs(wristEncoder.getPosition() - position) < 0.2;
  }

  /**Gets the voltage of the intake.
   * 
   * @return The voltage of the intake.
   */
  public double getIntakeVoltage() {
    return intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
  }

  //Putting intake and wrist data onto Elastic
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Intake");

    builder.addDoubleProperty("Intake Velocity", () -> getIntakeVelocity(), null);
    builder.addDoubleProperty("Intake Voltage", () -> getIntakeVoltage(), null);
    builder.addDoubleProperty("Intake Position", () -> intakeEncoder.getPosition(), null);
    builder.addDoubleProperty("Wrist Position", () -> getWristPosition(), null);
    builder.addDoubleProperty("Wrist Velocity", () -> getWristVelocity(), null);
    builder.addDoubleProperty("Wrist Voltage", () -> getWristVoltage(), null);
    builder.addDoubleProperty("Wrist Goal", () -> wristPID.getSetpoint(), null);
    builder.addBooleanProperty("Wrist Manual", () -> isManual(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
