// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;

import frc.robot.CustomXboxController;
import edu.wpi.first.wpilibj2.command.Command;

/*
 * This command is no longer in use. We've switched to using a method created
 * in the subsystem.
 */

/*
 * This command allows for the control of the wrist using a joystick.
 */

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveWristWithJoystick extends Command {
  private Intake intake;
  private CustomXboxController joystick;

  /** Creates a new MoveWristWithJoystick. */
  public MoveWristWithJoystick(Intake intake, CustomXboxController joystick) {
    this.intake = intake; //Make sure to use the "this" keyword when instantiating subsystems outside of RobotContainer.
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    /*
     * Make sure you add intake to the subsystem requirements. This will prevent
     * any other methods/commands from fighting this one while it is ongoing.
     */
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * Use the moveWrist() method from the intake subsystem.
     * Input the joystick left y value, and divide it by 5
     * to make sure the wrist doesn't move too fast.
     */
    intake.moveWrist(joystick.getLeftY()/5);
  }

/*
 * If we have to use this command, then something has gone wrong with
 * the regular wrist controls. This means that we don't need to put anything
 * to be run when the command ends, because we never expect this command to end.
 */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

/*
 * If this command is being used, something has happened to the regular
 * wrist controls. This means that we don't want this command to end once
 * it's been run, so we simply set the isFinished() method to always return false,
 * so the command never ends.
 */

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
