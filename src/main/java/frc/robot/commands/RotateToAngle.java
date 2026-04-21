// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngle extends Command {
  /** Creates a new RotateToAngle. */
  SwerveDrive swerve;

  double angle;

  PIDController rotatePID;

  public RotateToAngle(SwerveDrive swerve, double angle) {
    this.swerve = swerve;
    this.angle = angle;

    rotatePID = new PIDController(0.1, 0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotatePID.setTolerance(Units.degreesToRadians(2));
    rotatePID.enableContinuousInput(0, Units.degreesToRadians(360));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotatePID.setSetpoint(angle);

    double rotationSpeed = -rotatePID.calculate(swerve.gyroRad(), angle);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotationSpeed);

    swerve.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePID.atSetpoint();
  }
}
