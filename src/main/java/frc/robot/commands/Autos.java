// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

/*
 * The Autos class holds every single auto for the robot.
 * Many of the autos in this class are hard-coded as of current.
 * Typically, a pathplanning software would be used, but we could not
 * get PathPlanner to work consistently, so we just hard-coded
 * the autos.
 */

public final class Autos {
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

  /** *This auto currently does NOT work*
   * Runs a basic center auto that backs up, turns, and shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @param intake The intake subsystem.
   * @return A command that runs the basic center auto.
   */
  public static Command basicCenterAuto(SwerveDrive swerve, Shooter shooter, Intake intake){
    return Commands.sequence(
      /*
       * We want to set the pose of the robot at the start of the auto.
       * We can get this position from PathPlanner. Simply place the robot
       * at the desired starting position in the software, then grab the
       * coordinate values. Make sure that the starting position in real life
       * matches the starting position in the code.
       */
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 4.0, new Rotation2d(0)))),
      /*
       * When using PathPlanner, you can just import the autos
       * from the software.
       */
      new PathPlannerAuto("Basic Center Auto")
    ); 
  }

  /** *This auto currently does NOT work*
   * Runs a basic left auto that backs up, turns, and shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @return A command that runs the basic left auto.
   */
  public static Command basicLeftAuto(SwerveDrive swerve, Shooter shooter){
    return Commands.sequence(
      /*
       * We want to set the pose of the robot at the start of the auto.
       * We can get this position from PathPlanner. Simply place the robot
       * at the desired starting position in the software, then grab the
       * coordinate values. Make sure that the starting position in real life
       * matches the starting position in the code.
       */
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 5.574, new Rotation2d(0)))),
      /*
       * When using PathPlanner, you can just import the autos
       * from the software.
       */
      new PathPlannerAuto("Basic Left Auto")
    );
  }

  /** *This auto currently does NOT work*
   * Runs a basic right auto that backs up, turns, and shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @return A command that runs the basic right auto.
   */
  public static Command basicRightAuto(SwerveDrive swerve, Shooter shooter){
    /*
     * When using PathPlanner, you can just import the autos
     * from the software.
     */
    return new PathPlannerAuto("Basic Right Auto");
  }


  /** *This auto currently does NOT work*
   * Runs a right outpost auto that backs up to the outpost, obtains fuel from the outpost,
   * then shoots into the hub.
   * 
   * @param swerve The swerve subsystem.
   * @param shooter The shooter subsystem.
   * @return A command that runs the right outpost auto.
   */
  public static Command rightOutpostShoot(SwerveDrive swerve, Shooter shooter){
    return Commands.sequence(
      /*
       * We want to set the pose of the robot at the start of the auto.
       * We can get this position from PathPlanner. Simply place the robot
       * at the desired starting position in the software, then grab the
       * coordinate values. Make sure that the starting position in real life
       * matches the starting position in the code.
       */
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 0.666, new Rotation2d(0)))),
      /*
       * When using PathPlanner, you can just import the autos
       * from the software.
       */
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
      /*
       * We want to set the pose of the robot at the start of the auto.
       * We can get this position from PathPlanner. Simply place the robot
       * at the desired starting position in the software, then grab the
       * coordinate values. Make sure that the starting position in real life
       * matches the starting position in the code.
       */
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 1.791, new Rotation2d(0)))),
      /*
       * Turn the turret to the appropriate angle. This angle was obtained by
       * placing the robot at the starting position, then turning the turret to
       * the hub manually, then pulling the reading from Elastic. A timeout is used to
       * ensure that the method ends, which allows the rest of the sequence to run. 1.5
       * seconds is used as the timeout to allow time for the turret to fully turn.
       */
      Commands.run(() -> turret.turnTurret(120.11383056640625), turret).withTimeout(1.5),
      /*
       * Start shooting. 5 seconds should be enough to shoot all 8 pre-loaded fuel.
       * Since the starting position was set at the start of the auto, the calculations
       * for the turret position should be accurate, so those can be used.
       */
      Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).alongWith(shooter.delayedSpinSpindexer()).withTimeout(5),
      Commands.run(() -> turret.turnTurret(0), turret) //Return the turret to the starting position.
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
      /*
       * We want to set the pose of the robot at the start of the auto.
       * We can get this position from PathPlanner. Simply place the robot
       * at the desired starting position in the software, then grab the
       * coordinate values. Make sure that the starting position in real life
       * matches the starting position in the code.
       */
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.537, 6.209, new Rotation2d(0)))),
      /*
       * Our turret does not have 360 degrees of motion. It has, at most, maybe
       * 250 degrees of motion. Since there is a blind spot on the right side
       * of the robot, We need to move the robot backwards and rotate it. These values
       * were obtained through trial and error.
       */
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.drive(new ChassisSpeeds(0, 0, 0)), swerve).withTimeout(1.0),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0, 0, 2 * Math.PI), new Rotation2d(swerve.gyroRad()))), () -> swerve.drive(new ChassisSpeeds(0, 0, 0)), swerve).withTimeout(0.75),
      /*
       * Start shooting. 5 seconds should be enough to shoot all 8 pre-loaded fuel.
       * Since the starting position was set at the start of the auto, the calculations
       * for the turret position should be accurate, so those can be used.
       */
      Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).alongWith(shooter.delayedSpinSpindexer()).withTimeout(5)
    );
  }

  /**Runs a right side neutral zone auto. It backs up before going over the bump.
   * It intakes, then moves back over the bump. It then
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
      /*
       * We want to set the pose of the robot at the start of the auto.
       * We can get this position from PathPlanner. Simply place the robot
       * at the desired starting position in the software, then grab the
       * coordinate values. Make sure that the starting position in real life
       * matches the starting position in the code.
       */
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.568, 2.442, new Rotation2d(0)))),
      /*
       * Rather than shooting, we want to immediately go to the neutral zone
       * in order to get as much fuel as we can.
       */
      // Commands.parallel(
      //   Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(1.0),
      //   Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).alongWith(shooter.delayedSpinSpindexer()).withTimeout(3)
      // ),
      /*
       * Our robot has to go over the bump to get to the neutral zone. To do that,
       * we have to first back up, which allows the robot to build enough momentum
       * to consistently go over the bump. These values were tuned through trial and error.
       */
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-2.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(0.5),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(5.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(1.9),
      Commands.parallel(
        Commands.sequence(
          //Move the wrist down to the intaking position.
          Commands.run(() -> intake.wristToPosition(WRIST_DOWN_POSITION, controller), intake).withTimeout(0.5),
          Commands.parallel(
            /*
             * Begin intaking while moving forward. We don't want to move
             * too fast while intaking, as our intake is not able to handle
             * that much fuel at higher speeds.
             */
            Commands.runEnd(() -> intake.spinIntake(-1.0), () -> intake.stopIntake()).withTimeout(3.75),
            Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0.6, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(3.75)
          ),
          //Move the wrist to the middle position to prevent fuel from falling out.
          Commands.run(() -> intake.wristToPosition(WRIST_MIDDLE_POSITION, controller), intake).withTimeout(1.0)
        )
      ),
      /*
       * Move the robot back to the alliance zone, then rotate the
       * back of the robot to the hub. The reason we want to rotate specifically
       * the back of the robot to the hub is because the turret tracking
       * is most accurate when the back of the robot is facing the hub.
       */
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-5.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(2.15),
      
      //Rotate until the back of the robot is facing the hub; values from trial and error
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0.0, 0.0, Units.degreesToRadians(360)), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve)
        .alongWith(Commands.runEnd(() -> intake.spinIntake(-0.5), () -> intake.stopIntake()))
        .until(() -> ((swerve.gyroRad() % (2 * Math.PI)) > Units.degreesToRadians(225.0) && (swerve.gyroRad() % (2 * Math.PI)) < Units.degreesToRadians(235.0))),
      
      /*
       * Even though turret tracking is automatic, we want to double check
       * that the turret is at the correct position. To do this, we do the math
       * again and turn the turret to the calculated angle.
       */
      Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(0.5),
     
      /*
       * Start shooting. The calculations for the turret position/distance from hub
       * should be accurate, as the back of the robot has a camera. This
       * will update the vision readings before shooting begins, ensuring that
       * the shooter will have the correct velocities.
       */
      Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter)
        .alongWith(shooter.delayedSpinSpindexer()).withTimeout(9.0)
    );
  }

  /**Runs a left side neutral zone auto. It backs up before going over the bump.
   * It intakes, then moves back over the bump. It then
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
  public static Command LeftNeutralZoneAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision, Intake intake, CustomXboxController controller) {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.568, 5.536, new Rotation2d(0)))),
      // Commands.parallel(
      //   Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(1.0),
      //   Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).alongWith(shooter.delayedSpinSpindexer()).withTimeout(3)
      // ),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-2.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(0.5),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(5.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(1.9),
      Commands.parallel(
        Commands.sequence(
          Commands.run(() -> intake.wristToPosition(WRIST_DOWN_POSITION, controller), intake).withTimeout(0.5),
          Commands.parallel(
            Commands.runEnd(() -> intake.spinIntake(-1.0), () -> intake.stopIntake()).withTimeout(3.75),
            Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0.6, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(3.75)
          ),
          Commands.run(() -> intake.wristToPosition(WRIST_MIDDLE_POSITION, controller), intake).withTimeout(1.0)
        )
      ),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-5.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(2.15),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0.0, 0.0, Units.degreesToRadians(360)), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve)
        .alongWith(Commands.runEnd(() -> intake.spinIntake(-0.5), () -> intake.stopIntake()))
        .until(() -> ((swerve.gyroRad() % (2 * Math.PI)) > Units.degreesToRadians(125.0) && (swerve.gyroRad() % (2 * Math.PI)) < Units.degreesToRadians(135.0))),
      Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(0.5),
      Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter)
        .alongWith(shooter.delayedSpinSpindexer()).withTimeout(9.0)
    );
  }

  public static Command MiddleAuto(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision vision, Intake intake, CustomXboxController controller) {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setPose(new Pose2d(3.568, 4.012, new Rotation2d(0)))),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1.0, 0.0, 0.0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(2.5),
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0.0, 0.0, Units.degreesToRadians(360)), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules())
        .until(() -> ((swerve.gyroRad() % (2 * Math.PI)) > Units.degreesToRadians(175.0) && (swerve.gyroRad() % (2 * Math.PI)) < Units.degreesToRadians(185.0))),
      Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(0.5),
      Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter)
        .alongWith(shooter.delayedSpinSpindexer()).withTimeout(5.0)
    );
  }

  /** *This auto currently does NOT work*
   * Runs a depot auto. It backs up, turns 180 degrees, turns the turret, then shoots into the hub.
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
      /*
       * We want to set the pose of the robot at the start of the auto.
       * We can get this position from PathPlanner. Simply place the robot
       * at the desired starting position in the software, then grab the
       * coordinate values. Make sure that the starting position in real life
       * matches the starting position in the code.
       */
      Commands.runOnce(() -> swerve.setPose(FlippingUtil.flipFieldPose(new Pose2d(3.568, 6.050, new Rotation2d(0))))),
      //Drive backwards, then rotate 180 degrees. This will face the intake towards the depot.
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(1.0),
      new RotateToAngle(swerve, Units.degreesToRadians(180.0)).withTimeout(2.0),
      /*
       * Before intaking from the depot, turn the turret towards
       * the hub and shoot the pre-loaded fuel.
       */
      Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(1.0),
      Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).alongWith(shooter.delayedSpinSpindexer()).withTimeout(3),
      /*
       * Continue driving towards the depot. Once you've reached the
       * edge of the depot, bring the wrist down, start intaking, and drive over the fuel.
       */
      Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1.0, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(3.0),
      Commands.parallel(
        Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-0.5, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(2.0),
        Commands.run(() -> intake.wristToPosition(WRIST_DOWN_POSITION, controller), intake).withTimeout(0.5),
        Commands.runEnd(() -> intake.spinIntake(-0.95), () -> intake.stopIntake()).withTimeout(3)
      ),
      /*
       * Drive back towards the hub and bring the wrist
       * to the middle position to prevent fuel from falling out.
       */
      Commands.parallel(
        Commands.runEnd(() -> swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(1, 0, 0), new Rotation2d(swerve.gyroRad()))), () -> swerve.stopModules(), swerve).withTimeout(2.0),
        Commands.run(() -> intake.wristToPosition(WRIST_MIDDLE_POSITION, controller), intake).withTimeout(0.5)
      ),
      /*
       * Double check that the turret is turned to the correct position,
       * then start shooting. Calculated pose should be accurate,
       * as the cameras should be detecting tags at this position.
       */
      Commands.run(() -> turret.turnTurret(180 - (MathUtil.inputModulus(Math.atan2((vision.getHubCenterPose2d().getY() - swerve.getPose2d().getY()), (vision.getHubCenterPose2d().getX() - swerve.getPose2d().getX())), -180, 180)))).withTimeout(1.0),
      Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).alongWith(shooter.delayedSpinSpindexer()).withTimeout(5)
    );
  }

  /** *This auto currently does NOT work*
   * Runs a straight line auto in which the robot moves backwards in a straight line.
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
