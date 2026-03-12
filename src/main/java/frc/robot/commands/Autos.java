// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  /**Runs the SysIdRoutine of the top shooter motor.
   * 
   * @param shooter The shooter subsystem.
   * @return A command that runs the top shooter SysIdRoutine.
   */
  public static Command topShooterSysID(Shooter shooter) {
    return shooter.runTopShooterSysID();
  }

  /**Runs the SysIdRoutine of the bottom shooter motor.
   * 
   * @param shooter The shooter subsystem.
   * @return A command that runs the bottom shooter SysIdRoutine.
   */
  public static Command bottomShooterSysID(Shooter shooter) {
    return shooter.runBottombottomShooterSysID();
  }

  public static Command swerveSysID(SwerveDrive swerve) {
    return swerve.runSwerveSysID();
  }

  public static Command basicCenterAuto(SwerveDrive swerve, Shooter shooter, Intake intake){
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 4.0, new Rotation2d(0)))),
      new PathPlannerAuto("Basic Center Auto")
      // Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_MIDDLE_POSITION), intake).withTimeout(0.5),
      // Commands.run(() -> shooter.shootWithSpeed(-1500, -2200, 1), shooter)
      // Commands.run(() -> shooter.shootWithDistance(1), shooter).withTimeout(8)
    ); 
  }
  public static Command basicLeftAuto(SwerveDrive swerve, Shooter shooter){
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 5.574, new Rotation2d(0)))),
      new PathPlannerAuto("Basic Left Auto")
    );
  }

  public static Command basicRightAuto(SwerveDrive swerve, Shooter shooter){
    return new PathPlannerAuto("Basic Right Auto");
  }

  public static Command rightOutpostShoot(SwerveDrive swerve, Shooter shooter){
    return new PathPlannerAuto("Right Outpost Shoot");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
