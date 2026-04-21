// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {
  private SparkFlex wristMotor;
  private TalonFX intakeMotor;

  private RelativeEncoder wristEncoder;

  private SparkClosedLoopController wristPID;

  private TalonFXConfiguration intakeConfig;
  private SparkFlexConfig wristConfig;

  private boolean isManual;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new TalonFX(INTAKE_ID);
    wristMotor = new SparkFlex(WRIST_ID, MotorType.kBrushless);

    wristEncoder = wristMotor.getEncoder();

    isManual = false;

    wristPID = wristMotor.getClosedLoopController();

    intakeConfig = new TalonFXConfiguration();
    wristConfig = new SparkFlexConfig();

    intakeConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(60);

    intakeConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake);

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

    intakeMotor.getConfigurator().apply(intakeConfig);
    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**Spins the intake motor at the given speed.
   * 
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  public void spinIntake(double speed) {
    intakeMotor.set(speed);
  }

  /**Toggles manual mode for the wrist.
   * 
   */
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

  /**Moves the wrist with the joystick.
   * 
   * @param controller The controller to pull values from.
   */
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

  /**Returns whether the wrist is in manual mode or not.
   * 
   * @return {@code true} if manual mode is on, {@code false} otherwise.
   */
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

  //Putting intake and wrist data onto Elastic
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Intake");

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
