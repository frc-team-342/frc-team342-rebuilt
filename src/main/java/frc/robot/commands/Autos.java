// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import static frc.robot.Constants.IntakeConstants.WRIST_DOWN_POSITION;
import static frc.robot.Constants.IntakeConstants.WRIST_MIDDLE_POSITION;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.PhotonVision;
import frc.robot.CustomXboxController;

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

  /**Runs the SysIdRoutine for the SwerveDrive subsystem.
   * 
   * @param swerve The swerve subsystem.
   * @return A command that runs the SwerveDrive SysIdRoutine.
   */
  public static Command swerveSysID(SwerveDrive swerve) {
    return swerve.runSwerveSysID();
  }

  /**Runs a basic center auto that backs up, turns, and shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @param intake The intake subsystem.
   * @return A command that runs the basic center auto.
   */
  public static Command basicCenterAuto(SwerveDrive swerve, Shooter shooter, Intake intake){
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 4.0, new Rotation2d(0)))),
      new PathPlannerAuto("Basic Center Auto")
    ); 
  }

  /**Runs a basic left auto that backs up, turns, and shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @return A command that runs the basic left auto.
   */
  public static Command basicLeftAuto(SwerveDrive swerve, Shooter shooter){
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 5.574, new Rotation2d(0)))),
      new PathPlannerAuto("Basic Left Auto")
    );
  }

  /**Runs a basic right auto that backs up, turns, and shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @return A command that runs the basic right auto.
   */
  public static Command basicRightAuto(SwerveDrive swerve, Shooter shooter){
    return new PathPlannerAuto("Basic Right Auto");
  }


  /**Runs a right outpost auto that backs up to the outpost, obtains fuel from the outpost,
   * then shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @return A command that runs the right outpost auto.
   */
  public static Command rightOutpostShoot(SwerveDrive swerve, Shooter shooter){
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 0.666, new Rotation2d(0)))),
      new PathPlannerAuto("Right Outpost Shoot")
    );
  }

  /**Runs a basic right auto utilizing the turret. It turns the turret and shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @param turret The turret subsystem.
   * @param vision The vision subsystem.
   * @return A command that runs the basic right turret auto.
   */
  public static Command basicRightTurretAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision) {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 1.791, new Rotation2d(0)))),
      Commands.run(() -> turret.turnTurret(120.11383056640625), turret).withTimeout(1.5),
      Commands.runEnd(() -> shooter.shootWithDistance(1, vision.getHubCenterPose2d()), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(5),
      Commands.run(() -> turret.turnTurret(0), turret)
    );
  }

  /**Runs a basic left auto utilizing the turret. It backs up, rotates, turns the turret,
   * then shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @param turret The turret subsystem.
   * @param vision The vision subsystem.
   * @return A command that runs the basic left turret auto.
   */
  public static Command basicLeftTurretAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision) {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 6.209, new Rotation2d(0)))),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1.0, 0, 2 * Math.PI), new Rotation2d(swerve.gyroRad()))), () -> swerve.drive(new ChassisSpeeds(0, 0, 0)), swerve).withTimeout(1),
      Commands.runEnd(() -> shooter.shootWithDistance(1, vision.getHubCenterPose2d()), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(5)
    );
  }

  /**Runs a right side neutral zone auto. It turns the turret and shoots into the hub and then
   * backs up before going over the bump. It intakes, then moves back over the bump. It then
   * turns the robot, turns the turret, and shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @param turret The turret subsystem.
   * @param vision The vision subsystem.
   * @param intake The intake subsystem.
   * @param controller The operator controller.
   * @return A command that runs the right neutral zone auto.
   */
  public static Command rightNeutralZoneAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision, Intake intake, CustomXboxController controller) {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.568, 2.884, new Rotation2d(0)))),
      Commands.parallel(
        Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(1.0),
        Commands.runEnd(() -> shooter.shootWithDistance(1, vision.getHubCenterPose2d()), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(3)
      ),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(1.0),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(5.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(1.5),
      Commands.parallel(
        Commands.sequence(
          Commands.run(() -> intake.wristToPosition(WRIST_DOWN_POSITION, controller), intake).withTimeout(0.5),
          Commands.parallel(
            Commands.runEnd(() -> intake.spinIntake(-0.95), () -> intake.stopIntake()).withTimeout(6.0),
            Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0.5, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(6.0)
          ),
          Commands.run(() -> intake.wristToPosition(WRIST_MIDDLE_POSITION, controller), intake).withTimeout(1.0)
        )
      ),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-5.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(2),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0.0, 0.0, Units.degreesToRadians(90)), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).until(() -> (swerve.gyroRad() > Units.degreesToRadians(150.0) && swerve.gyroRad() < Units.degreesToRadians(210.0))),
      Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(1.0),
      Commands.runEnd(() -> shooter.shootWithDistance(1, vision.getHubCenterPose2d()), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(3)
    );
  }

  /**Runs a depot auto. It backs up, turns 180 degrees, turns the turret, then shoots into the hub.
   * It then intakes from the depot, drives forward, turns the turret, then shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @param turret The turret subsystem.
   * @param vision The vision subsystem.
   * @param intake The intake subsystem.
   * @param controller The operator controller.
   * @return A command that runs the depot auto.
   */
  public static Command depotAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision, Intake intake, CustomXboxController controller) {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(FlippingUtil.flipFieldPose(new Pose2d(3.568, 6.050, new Rotation2d(0))))),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(1.0),
      new RotateToAngle(swerve, Units.degreesToRadians(180.0)).withTimeout(2.0),
      Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(1.0),
      Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(3),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(3.0),
      Commands.parallel(
        Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-0.5, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(2.0),
        Commands.run(() -> intake.wristToPosition(WRIST_DOWN_POSITION, controller), intake).withTimeout(0.5),
        Commands.runEnd(() -> intake.spinIntake(-0.95), () -> intake.stopIntake()).withTimeout(3)
      ),
      Commands.parallel(
        Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(1, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(2.0),
        Commands.run(() -> intake.wristToPosition(WRIST_MIDDLE_POSITION, controller), intake).withTimeout(0.5)
      ),
      Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(1.0),
      Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(5)
    );
  }

  /**Runs a straight line auto in which the robot moves backwards in a straight line.
   * 
   * @param swerve The swerve subsystem.
   * @return A command that runs the straight line auto.
   */
  public static Command straightLineAuto(SwerveDrive swerve) {
    return new PathPlannerAuto("Straight Line Auto");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
