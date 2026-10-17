// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

/*
 * This command is used to control the drive train with a joystick.
 */

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithJoystick extends Command {
  /** Creates a new DriveWithJoystick. */
  private SwerveDrive swerve;
  private XboxController joyStick;
  private ChassisSpeeds chassisSpeeds;

  public DriveWithJoystick(SwerveDrive swerve, XboxController joyStick) {
    this.swerve = swerve; //Make sure to always use "this" to instantiate subsystems outside of RobotContainer.
    this.joyStick = joyStick;
    addRequirements(swerve);
    /*
     * This will be the default command for the swerve subsystem, so make
     * sure you add the swerve subsystem as a requirement for this command.
     */
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * Context for how coordinates work in FRC:
     * 
     * The x-axis on a FRC field is right/left (right is positive).
     * The y-axis on a FRC field is forward/backward (forward is positive).
     * All directions mentioned are relative to the driver station wall.
     */

    /* Gets values from the Left(Drive) on the Xbox controller */
    double xSpeed = joyStick.getLeftY(); //Y on the joystick corresponds to forward/backward on the field.
    double ySpeed = joyStick.getLeftX(); //X on the joystick corresponds to right/left on the field.
    /*
     * Axis 4 refers to the right joystick. We don't want to pull
     * specifically x or y values from the right joystick, so we instead
     * pull the value from the right joystick using getRawAxis().
     */
    double rotateSpeed = joyStick.getRawAxis(4);
    /*
     * The leftTriggerValue variable is used as a speed modifier. This
     * is used for slow mode, which does what it's called: it slows down the
     * drive and rotation speed of the robot. This allows the driver to make
     * more precise maneuvers when needed.
     */
    double leftTriggerValue = joyStick.getLeftTriggerAxis();

    /*
     * Calculate the speed/rotate modifier.
     * Subtract the minimum speed from the maximum speed.
     * Multiply this by the leftTriggerValue. When the left trigger
     * is not pressed, it returns a value of 0.0., meaning the
     * speed/rotate modifiers are just the maximum speeds.
     * If the left trigger is pressed, then it decreases the modifier
     * as the left trigger is pressed farther down.
     */
    double speedModifier = DriveConstants.MAX_DRIVE_SPEED - (leftTriggerValue * (DriveConstants.MAX_DRIVE_SPEED - DriveConstants.MIN_DRIVE_SPEED));
    double rotateModifier = DriveConstants.MAX_ROTATE_SPEED - (leftTriggerValue * (DriveConstants.MAX_ROTATE_SPEED - DriveConstants.MIN_ROTATE_SPPEED));

    /*Applies deadband */
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.15);
    rotateSpeed = MathUtil.applyDeadband(rotateSpeed, 0.15);

    /*
     * Multiply the xSpeed by the speedModifier. This will
     * give you your vx of your robot.
     * Remember: xSpeed (and ySpeed and rotateSpeed, as well) is holding a value from [-1, 1].
     */
    xSpeed = xSpeed * speedModifier;
    ySpeed = ySpeed * speedModifier; //Do the same for ySpeed
    rotateSpeed = rotateSpeed * rotateModifier; //Multiply rotateSpeed and rotateModifier

    /*
     * Make sure to add negative in front of all the calculated values.
     * This is due to how the values from the joysticks are read. Up and right
     * give negative values. This means that the joysticks would be inverted.
     * To fix this, simply add negatives in front of the all calculated velocities.
     */

    /* Puts the x,y, and rotates speeds into a new ChassisSpeeds */
    chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, -rotateSpeed);

    /* Passes through the Chassisspeeds just created into the Drive Method */
    swerve.drive(chassisSpeeds);
  }

  /*
   * Once this command ends, we want the robot to stop moving.
   * This means that we just want to stop all the swerve modules.
   */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  /*
   * The only time this method should end is when the robot is disabled.
   * Therefore, we can just return false for this method.
   */

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
