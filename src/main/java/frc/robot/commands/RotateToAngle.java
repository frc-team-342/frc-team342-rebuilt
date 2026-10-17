// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

//This command is currently NOT being used.

/*
 * This command is used to rotate the robot to a certain angle. You technically are able to create
 * this as a method then use lambdas, but we decided to create it as a command file instead.
 */

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngle extends Command {
  /** Creates a new RotateToAngle. */
  SwerveDrive swerve;

  double angle;

  PIDController rotatePID;

  public RotateToAngle(SwerveDrive swerve, double angle) {
    this.swerve = swerve; //Make sure you use the "this" keyword to instantiate the swerve subsystem
    this.angle = angle;

    rotatePID = new PIDController(0.1, 0, 0); //You'll need to create a PID controller to rotate to an angle

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    /*
     * Use addRequirements to add a subsystem requirement. This prevents other methods/commands from
     * using the subsystem, which prevents methods/commands from conflicting with each other.
     */
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
     * Set the amount of error allowed. The method takes in radians,
     * but we don't know how many radians are in 2 degrees. So, we just used
     * the Units.degreesToRadians() method, which converts from degrees
     * to radians for us.
     */
    rotatePID.setTolerance(Units.degreesToRadians(2));

    /*
     * Set continuous input to 0-360. This means that, rather than treating 0 and 360 degrees as
     * hard stops, it considers them as the same point. This allows the PID to calculate
     * the shortest route to the setpoint, rather than only calculating in one direction.
     */
    rotatePID.enableContinuousInput(0, Units.degreesToRadians(360));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotatePID.setSetpoint(angle); //Set the setpoint to the desired angle.

    /*
     * Calculate the rotation speed of the robot. The .calculate() method
     * takes the current angle and the desired angle, does some math, and returns
     * a rotation speed. The negative is added to ensure the robot defaults to
     * clockwise rotation.
     */
    double rotationSpeed = -rotatePID.calculate(swerve.gyroRad(), angle);

    /*
     * Create a new ChassisSpeeds object. Input 0 for vx and vy, because
     * we don't want the robot to move. Input the calculated rotation speed
     * for omegaRadiansPerSecond.
     */
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotationSpeed);

    /*
     * Once you've created a ChassisSpeeds object with the approriate
     * values, use the drive method in the SwerveDrive subsystem. Input
     * the ChassisSpeeds object you just created.
     */
    swerve.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*
     * Once the command ends, we want the PID controllers to stop holding
     * the inputted angle, as we want to be able to rotate even after using
     * this command. To do this, we simply stop all the modules after the command ends.
     */
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * This condition should be true when you want the command to end.
     * We want the command to end once the robot has reached the desired
     * angle. Simply use the .atSetpoint() method.
     */
    return rotatePID.atSetpoint();
  }
}
