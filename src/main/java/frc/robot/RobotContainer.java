// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwereDrive;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwereDrive swere;
  private final Turret turret;
  private final PhotonVision photonVision;

  private final XboxController driver;
  private final XboxController operator;

  private final JoystickButton fieldOrientedButton;
  private final JoystickButton turretToggleButton;

  private final Command toggleFieldOriented;
  private final Command turretToAngle;
  private final Command toggleManualTurret;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    photonVision = new PhotonVision();

    swere = new SwereDrive();
    turret = new Turret();

    driver = new XboxController(0);
    operator = new XboxController(1);

    toggleFieldOriented = Commands.runOnce(() -> {swere.toggleFieldOriented();}, swere);
    toggleManualTurret = Commands.runOnce(() -> {turret.toggleManual();}, turret);
    turretToAngle = Commands.run(() -> {turret.turretToAngle(0, operator);});

    fieldOrientedButton = new JoystickButton(driver, XboxController.Button.kA.value);
    turretToggleButton = new JoystickButton(operator, XboxController.Button.kStart.value);

    swere.setDefaultCommand(swere.driveWithJoystick(driver));
    turret.setDefaultCommand(turretToAngle);

    SmartDashboard.putData(swere);
    SmartDashboard.putData(turret);
    SmartDashboard.putData(photonVision);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    fieldOrientedButton.onTrue(toggleFieldOriented); // 'A' button
    turretToggleButton.onTrue(toggleManualTurret); // 'Start' button
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
