// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Spindexer extends SubsystemBase {
  private SparkFlex spindexer;
  private SparkFlexConfig spindexerConfig;
  
  private boolean isShooting;
  private boolean shooterUpToSpeed;
  /** Creates a new Spindexer. */
  public Spindexer() {
    spindexer = new SparkFlex(SPINDEXER_ID, MotorType.kBrushless);
    spindexerConfig = new SparkFlexConfig();

    spindexerConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(60);

      spindexer.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    isShooting = false;
    shooterUpToSpeed = false;
  }

  /**
   * @param Shooting updates the isShooting instance variable to whether or not the shooter is running
   */
  public void setShooting(boolean shooting){
    isShooting = shooting;
  }

  /**
   * @return isShooting: whether or not the shooter is currently running
   */
  public boolean getShooting(){
    return isShooting;
  }

  /**Sets shooterUpToSpeed to the inputted boolean value.
   * 
   */
  public void setAtDesiredSpeed(boolean atDesiredSpeed) {
    shooterUpToSpeed = atDesiredSpeed;
  }

  /**Checks if the shooter is at the desired speed.
   * 
   * @return {@code true} if the shooter is up to speed, {@code false} otherwise.
   */
  public boolean AtDesiredSpeed() {
    return shooterUpToSpeed;
  }

  /**
   * Spins the spindexer if the shooter is running.
   */
  public void spinSpindexer(){
    if(isShooting) {
      spindexer.set(0.7);
    }
    else{
      spindexer.set(0);
    }
  }

  /**Spins the spindexer at a set speed.
   * 
   * @param speed The speed to spin the spindexer at.
   */
  public void SpindexerWithSpeed(double speed) {
    spindexer.set(speed);
  }

  /**
   * @return Returns a command for running the spindexer
   */
  public Command runSpindexer(){
    return new WaitCommand(0.75).andThen(() -> spinSpindexer(), this);
  }

  public double getSpindexerVelocity() {
    return spindexer.getEncoder().getVelocity();
  }

  /**
   * Puts data for the spindexer onto smartdashboard
   */
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addBooleanProperty("Is shooting", () -> getShooting(), null);
    builder.addDoubleProperty("Spindexer Velocity", () -> getSpindexerVelocity(), null);
  }

  @Override
  public void periodic() {
    // spindexer.set(0.1);
  }
}
