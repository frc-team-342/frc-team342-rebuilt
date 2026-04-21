// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.IntakeConstants.WRIST_DOWN_POSITION;
import static frc.robot.Constants.IntakeConstants.WRIST_MIDDLE_POSITION;
import static frc.robot.Constants.IntakeConstants.WRIST_UP_POSITION;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.PhotonVision;
import frc.robot.CustomXboxController;
import frc.robot.Constants.IntakeConstants;

public final class Autos {
  private static PathConstraints globalConstraints = new PathConstraints(1, 1.5, 180, 240);

  // private static PathPlannerPath rightToNeutralZone; {
  //   try{
  //     rightToNeutralZone = PathPlannerPath.fromPathFile("Right to Neutral Zone");
  //   }catch(Exception e) {
  //     DriverStation.reportError("Failed to load Right To Neutral Zone path", e.getStackTrace());
  //     throw new RuntimeException(e);
  //   }
  // }

  // private static PathPlannerPath neutralZoneToShoot; {
  //   try{
  //     neutralZoneToShoot = PathPlannerPath.fromPathFile("Neutral Zone to Shoot");
  //   }catch(Exception e) {
  //     DriverStation.reportError("Failed to load Neutral Zone to Shoot path", e.getStackTrace());
  //     throw new RuntimeException(e);
  //   }
  // }

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
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 0.666, new Rotation2d(0)))),
      new PathPlannerAuto("Right Outpost Shoot")
    );
  }

  public static Command basicRightTurretAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision, CustomXboxController controller) {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 1.791, new Rotation2d(0)))),
      // Commands.runOnce(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)), controller)).withTimeout(1.0),
      Commands.run(() -> turret.turnTurret(120.11383056640625), turret).withTimeout(1.5),
      Commands.runEnd(() -> shooter.shootWithDistance(1, vision.getHubCenterPose2d()), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(5),
      Commands.run(() -> turret.turnTurret(0), turret)
    );
  }

  public static Command basicLeftTurretAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision, CustomXboxController controller) {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 6.209, new Rotation2d(0)))),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1.0, 0, 2 * Math.PI), new Rotation2d(swerve.gyroRad()))), () -> swerve.drive(new ChassisSpeeds(0, 0, 0)), swerve).withTimeout(1),
      Commands.runEnd(() -> shooter.shootWithDistance(1, vision.getHubCenterPose2d()), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(5)
    );
  }

  public static Command rightNeutralZoneAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision, Intake intake, CustomXboxController controller) {
    // return Commands.sequence(
    //   Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 2.442, new Rotation2d(0)))),
    //   AutoBuilder.pathfindThenFollowPath(rightToNeutralZone, new PathConstraints(1.0, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(600))),
    //   Commands.runOnce(() -> swerve.setPose(swerve.getPose2d())),
    //   AutoBuilder.pathfindThenFollowPath(neutralZoneToShoot, new PathConstraints(1.0, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(600)))
    // );

    // return Commands.sequence(
    //   Commands.runOnce(() -> swerve.setPose(FlippingUtil.flipFieldPose(new Pose2d(3.568, 2.442, new Rotation2d(0))))),
    //   Commands.runOnce(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)), controller)),
    //   Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(3),
    //   AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.568, 2.442, new Rotation2d(0)), globalConstraints),
    //   AutoBuilder.pathfindToPoseFlipped(new Pose2d(6.363, 2.442, new Rotation2d(0)), new PathConstraints(2.0, 3.0, Units.degreesToRadians(90), Units.degreesToRadians(180))),
    //   Commands.parallel(
    //     Commands.run(() -> intake.wristToPosition(WRIST_DOWN_POSITION, controller), intake).withTimeout(0.5),
    //     Commands.runEnd(() -> intake.spinIntake(-1), () -> intake.stopIntake()).withTimeout(3),
    //     AutoBuilder.pathfindToPoseFlipped(new Pose2d(7.212, 2.442, new Rotation2d(0)), globalConstraints)
    //   ),
    //   Commands.parallel(
    //     Commands.run(() -> intake.wristToPosition(WRIST_MIDDLE_POSITION, controller), intake).withTimeout(0.5),
    //     AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.568, 2.442, new Rotation2d(0)), new PathConstraints(2.0, 3.0, Units.degreesToRadians(90), Units.degreesToRadians(180)))
    //   ),
    //   Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(5)
    // );

    // return new PathPlannerAuto("Right Neutral Zone Auto");

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

  public static Command depotAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision, Intake intake, CustomXboxController controller) {
    // return Commands.sequence(
    //   Commands.runOnce(() -> swerve.setPose(FlippingUtil.flipFieldPose(new Pose2d(3.568, 6.050, new Rotation2d(0))))),
    //   AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.568, 6.050, new Rotation2d(180)), globalConstraints),
    //   Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(3),
    //   AutoBuilder.pathfindToPoseFlipped(new Pose2d(1.568, 6.050, new Rotation2d(180)), globalConstraints),
    //   Commands.parallel(
    //     AutoBuilder.pathfindToPoseFlipped(new Pose2d(0.568, 6.050, new Rotation2d(180)), globalConstraints),
    //     Commands.run(() -> intake.wristToPosition(WRIST_DOWN_POSITION, controller), intake).withTimeout(0.5),
    //     Commands.runEnd(() -> intake.spinIntake(-1), () -> intake.stopIntake()).withTimeout(3)
    //   ),
    //   Commands.parallel(
    //     AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.568, 6.050, new Rotation2d(180)), globalConstraints),
    //     Commands.run((() -> intake.wristToPosition(WRIST_MIDDLE_POSITION, controller)), intake).withTimeout(0.5)
    //   ),
    //   Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(5)
    // );

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

    // return new PathPlannerAuto("Depot Auto");
  }

  public static Command straightLineAuto(SwerveDrive swerve) {
    return new PathPlannerAuto("Straight Line Auto");
    // return Commands.sequence(
    //   Commands.runOnce(() -> swerve.setPose(FlippingUtil.flipFieldPose(new Pose2d(3.537, 2.442, new Rotation2d(0)))), swerve),
    //   AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.537, 2.442, new Rotation2d(0)), globalConstraints)
    // );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
