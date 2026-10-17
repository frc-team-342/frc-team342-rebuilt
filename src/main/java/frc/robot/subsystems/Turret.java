// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  private SparkFlex turretMotor;
  private SparkFlexConfig turretConfig;
  private RelativeEncoder turretEncoder;
  private SparkClosedLoopController turretController;

  /*
   * The turret subsystem requires the swerve, vision, and shooter
   * subsystems to work properly.
   */
  private SwerveDrive swerve;
	private PhotonVision vision;
  private Shooter shooter;

  private double goal; //A variable to hold the goal position of the turret.

  /** Creates a new Turret. */
  public Turret(SwerveDrive swerve, PhotonVision vision, Shooter shooter) {
    /*
     * The reason we instantiate the swerve, vision, and shooter subsystems using
     * the "this" keyword and parameters is because you are only allowed to
     * create one instance of each subsystem. This prevents us from creating
     * instances of these subsystems inside of the turret subsystem, so we
     * have to instead instantiate them using parameters.
     */
    this.swerve = swerve;
		this.vision = vision;
    this.shooter = shooter;

    turretMotor = new SparkFlex(TurretConstants.TURRET_ID, MotorType.kBrushless);
    turretConfig = new SparkFlexConfig();
    turretEncoder = turretMotor.getEncoder();
    turretController = turretMotor.getClosedLoopController();

    turretConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    //This converts the turret encoder reading from rotations to degrees.
    turretConfig.encoder
      .positionConversionFactor(TurretConstants.TURRET_POSITION_CONVERSION);

    /*
     * The turret PID controller uses two different slots. This is because, depending on
     * the position and direction of rotation of the turret, the spring mechanism
     * puts up different amounts of resistance. To prevent the turret from overshooting
     * or undershooting consistently, we decided to use two different slots that
     * contain different PID values.
     */
    turretConfig.closedLoop
      .p(TurretConstants.TURRET_PID_VALUES_SLOT0[0], ClosedLoopSlot.kSlot0)
      .i(TurretConstants.TURRET_PID_VALUES_SLOT0[1], ClosedLoopSlot.kSlot0)
      .d(TurretConstants.TURRET_PID_VALUES_SLOT0[2], ClosedLoopSlot.kSlot0)
			.p(TURRET_PID_VALUES_SLOT1[0], ClosedLoopSlot.kSlot1)
			.i(TURRET_PID_VALUES_SLOT1[1], ClosedLoopSlot.kSlot1)
			.d(TURRET_PID_VALUES_SLOT1[2], ClosedLoopSlot.kSlot1)
      .positionWrappingEnabled(false)
			.allowedClosedLoopError(TURRET_ALLOWED_ERROR, ClosedLoopSlot.kSlot0)
      .allowedClosedLoopError(TURRET_ALLOWED_ERROR, ClosedLoopSlot.kSlot1);

    //Remember to apply the config
    turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    goal = 0.0; //Always instantiate the goal to 0, as we want the turret to start at 0.
  }

  /**Tells the turret to rotate to the angle to the pose from a lookahead pose.
   * 
   * @param pose The pose to get the angle to.
   */
  public void trackLookAheadPose(Pose2d pose) {
    /*
     * Because of the way we calculate the angle to which the turret needs to turn to,
     * it can calculate an angle that is out of the turret's range of motion. To ensure
     * the turret does not rip itself off the robot, we want to clamp the goal angle.
     */
		double safeGoal = MathUtil.clamp(angleToLookAheadPose(pose), TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
    goal = safeGoal; //Set the goal angle to the clamped goal angle.

    //Check if the turret's current position is more than 40 degrees counterclockwise.
		if(turretEncoder.getPosition() < -40) {
      //If it is, use the stronger PID, because there's more resistance.
			turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    //Check if the turret's current position is greater than the goal position (turning counterclockwise).
		}else if(turretEncoder.getPosition() > goal) {
      //If it is, use the weaker PID, because there's less resistance.
    	turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    //If the turret position is less than 40 degrees counterclockwise and it's rotating clockwise
		}else{
      //Use the stronger PID, because there's more resistance
			turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
		}
  }

  /**Turns the turret to a setpoint.
   * 
   * @param setpoint The angle to turn the turret to.
   */
  public void turnTurret(double setpoint) {
    //Same logic as in trackLookAheadPose.
		if(turretEncoder.getPosition() < -40) {
			turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot1);
		}else if(Math.abs(turretEncoder.getPosition()) > setpoint) {
    	turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
		}else if(Math.abs(turretEncoder.getPosition()) < setpoint) {
			turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot1);
		}
  }

  /**Sets the turret motor speed to 0 */
  public void stop(){
    turretMotor.set(0);
  }

  /**Gets the lookahead poses for the robot and the turret.
   * 
   * @return An array containing the lookahead poses for the robot and turret.
   */
  public Pose2d[] getLookAheadPoses() {
    //We grab the current robot pose from the SwerveDrivePoseEstimator.
    Pose2d currentPose = swerve.getPose2d();

    /*
     * We get the x and y displacements. We get the x displacement by taking the speed
     * of the robot on the x axis and multiplying that by the flight time of the fuel
     * at the current distance from the hub. We do the same calculations with the y axis
     * speed to get the y displacement.
     */
    double xDisplacement = swerve.getChassisSpeeds().vxMetersPerSecond * shooter.getFlightTime(vision.getDistanceToHub(currentPose));
    double yDisplacement = swerve.getChassisSpeeds().vyMetersPerSecond * shooter.getFlightTime(vision.getDistanceToHub(currentPose));

    /*
     * Calculate the future x and y coordinates of the robot by adding the
     * x displacement and y displacement to the current x and y coordinates, respectively.
     */
    double displacedX = currentPose.getX() + xDisplacement;
    double displacedY = currentPose.getY() + yDisplacement;

    /*
     * Declare and instantiate a variable that holds the future robot pose.
     * Keep the rotation the same as the current rotation. Get the future turret pose
     * by adding the turret positional offset to the future robot pose.
     */
    Pose2d nextRobotPose = new Pose2d(displacedX, displacedY, new Rotation2d(currentPose.getRotation().getRadians()));
    Pose2d nextTurretPose = nextRobotPose.plus(TURRET_OFFSET);

    // Create an array to hold both the future robot and turret poses.
    Pose2d[] lookAheadPoses = {nextRobotPose, nextTurretPose};

    //Return the array.
    return lookAheadPoses;
  }

  /**Gets the angle to the given pose from the lookahead pose of the turret.
   * 
   * @param pose The pose to get the angle to.
   * @return The angle to the given pose from the lookahead pose of the turret.
   */
  public double angleToLookAheadPose(Pose2d pose) {
    /*
     * To calculate the angle of the turret to the given pose, we need
     * the y and x distance from the given pose to the turret. We pull the
     * pose of the turret from the array returned by the getLookAheadPoses method,
     * then subtract the y and x from the y and x of the given pose.
     */
    double yDistance = pose.getY() - getLookAheadPoses()[1].getY();
    double xDistance = pose.getX() - getLookAheadPoses()[1].getX();

    /*
     * We calculate the angle the turret needs to turn to in order to
     * reach the given pose. We do this by using arctan.
     */
    Rotation2d angleToTarget = Rotation2d.fromRadians(Math.atan2(yDistance, xDistance));

    /*
     * We then want to find out what the counterclockwise and clockwise angles would be.
     * The reason we have a 180 at the front of both statements is because our turret's
     * starting rotation is 180 degrees away from our robot's starting position. Since the
     * turret's pose is calculated with the same rotation as the robot, we account for it by
     * adding or subtracting 180 to each calculation. We then take the angle to the target in
     * degrees, and then we subtract the robot's rotation from it. We already accounted for the
     * starting rotation discrepancy between the robot and turret, so we can treat the robot
     * rotation as the turret rotation now. By subtracting the robot rotation from the target
     * angle, we are able to figure out the counterclockwise and clockwise angles that
     * the turret would have to turn to in order to reach the goal angle.
     */
    double ccwDesiredTurretAngle = -(180 + (MathUtil.inputModulus(angleToTarget.getDegrees(), -180, 180) - MathUtil.inputModulus(getLookAheadPoses()[0].getRotation().getDegrees(), -180, 180)));
    double cwDesiredTurretAngle = 180 - (MathUtil.inputModulus(angleToTarget.getDegrees(), -180, 180) - MathUtil.inputModulus(getLookAheadPoses()[0].getRotation().getDegrees(), -180, 180));

    /*
     * In order to figure out which angle is the fastest to turn to, we
     * need to figure out the distance of each angle from the starting
     * turret position. We subtract each angle from 0, then take the absolute
     * value to figure out the distance to each angle.
     */
    double ccwAngleDistance = Math.abs(0 - ccwDesiredTurretAngle);
    double cwAngleDistance = Math.abs(0 - cwDesiredTurretAngle);

    /*
     * Check if the desired counterclockwise angle is less than the minimum possible turret angle.
     * Check if the desired clockwise angle is less than the maximum possible turret angle.
     * If both are true, set the goal angle to the desired clockwise angle.
     */
    if(ccwDesiredTurretAngle < TURRET_MIN_ANGLE && cwDesiredTurretAngle < TURRET_MAX_ANGLE) {
      goal = cwDesiredTurretAngle;
    /*
     * Same logic as above but flipped.
     */
    }else if(ccwDesiredTurretAngle > TURRET_MIN_ANGLE && cwDesiredTurretAngle > TURRET_MAX_ANGLE) {
      goal = ccwDesiredTurretAngle;
    /*
     * Check if both the desired counterclockwise and clockwise angles are within the turret
     * range of motions. If both are, check which distance is shorter.
     */
    }else if(ccwDesiredTurretAngle > TURRET_MIN_ANGLE && cwDesiredTurretAngle < TURRET_MAX_ANGLE) {
      /*
       * Ternary operator; it works like a if-else statement.
       * If the distance to the desired counterclockwise angle is shorter than
       * the distance to the desired clockwise angle, then set the goal angle to
       * the desired counterclockwise angle. Otherwise, set the goal angle to
       * the desired clockwise angle.
       */
      goal = ccwAngleDistance < cwAngleDistance ? ccwDesiredTurretAngle : cwDesiredTurretAngle;
    }else{
      /*
       * If none of the logic above is met, then simply set the goal to the
       * current position. This is to prevent unnecessary turret movement.
       */
      goal = turretEncoder.getPosition();
    }

    return goal; //Return the goal angle.
  }

  /**puts the data for the turret on smartdashboard*/
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Turret Position", () -> turretEncoder.getPosition(), null);
    builder.addDoubleProperty("Turret PID Setpoint", () -> turretController.getSetpoint(), null);
    builder.addDoubleProperty("Turret Assumed Goal", () -> this.goal, null);
    builder.addDoubleProperty("Turret Distance to Hub", () -> vision.getDistanceToHub(getLookAheadPoses()[1]), null);
    builder.addDoubleProperty("Turret Lookahead X", () -> getLookAheadPoses()[1].getX(), null);
    builder.addDoubleProperty("Turret Lookahead Y", () -> getLookAheadPoses()[1].getY(), null);
    builder.addDoubleProperty("Interpolated Top Shooter Velocity", () -> shooter.getTopTargetVelocity(vision.getDistanceToHub(getLookAheadPoses()[1])), null);
    builder.addDoubleProperty("Interpolated Bottom Shooter Velocity", () -> shooter.getBottomTargetVelocity(vision.getDistanceToHub(getLookAheadPoses()[1])), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
