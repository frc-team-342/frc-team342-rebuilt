// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAhead;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveWristWithJoystick;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.IntakeConstants.*;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CustomXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrive swere;
  private final Turret turret;
  private final PhotonVision photonVision;
  private final Intake intake;
  private final Spindexer spindexer;
  private final Shooter shooter;

  private final CustomXboxController driver;
  private final CustomXboxController operator;

  private final JoystickButton fieldOrientedButton;
  private final JoystickButton turretToggleButton;
  private final JoystickButton toggleWristButton;
  private final JoystickButton turretFastModeToggleButton;
  private final JoystickButton shootButton;
  // private final JoystickButton reverseIntakeButton;
  private final JoystickButton toggleDriveAssistButton;
  private final JoystickButton downtakeButton;
  // private final JoystickButton intakeButton;
  private final JoystickButton straightAheadTurretButton;
  private final JoystickButton leftSideTurretTurnButton;
  private final JoystickButton rightSideTurretTurnButton;
  private final POVButton wristDownButton;
  private final POVButton wristUpButton;
  private final POVButton wristMiddleButton;

  private final DriveWithJoystick driveWithJoystick;
  private final MoveWristWithJoystick moveWristWithJoystick;

  private final Command toggleFieldOriented;
  // private final Command turretToAngle;
  private final Command toggleManualTurret;
  private final Command toggleFastTurret;
  private final Command toggleDriveAssist;
  private final Command wristDown;
  private final Command wristUp;
  private final Command wristMiddle;
  private final Command intakeFuel;
  private final Command getFuelUnstuck;
  private final Command shoot;
  private final Command turretShoot;
  private final Command downtake;
  private final Command shootWhileMoving;
  private final Command toggleWristManual;

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    photonVision = new PhotonVision();
    swere = new SwerveDrive(photonVision);
    turret = new Turret(swere, photonVision);
    intake = new Intake();
    spindexer = new Spindexer();
    shooter = new Shooter(spindexer, photonVision);

    driver = new CustomXboxController(0);
    operator = new CustomXboxController(1);

    toggleFieldOriented = Commands.runOnce(() -> {swere.toggleFieldOriented();}, swere);
    toggleManualTurret = Commands.runOnce(() -> {turret.toggleManual();});
    toggleDriveAssist = Commands.runOnce(() -> {swere.toggleDriveAssist();}, swere);
    toggleWristManual = Commands.runOnce(() -> intake.toggleManual());
    toggleFastTurret = Commands.runOnce(() -> turret.toggleFastTurret());
    // turretToAngle = new AimAhead(swere, shooter, null, photonVision, driver);
    // turretToAngle = Commands.run(() -> {turret.turretToAngle(turret.getAngleToPose(photonVision.getHubCenterPose2d()), operator);}, turret);
    wristDown = Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_DOWN_POSITION, operator), intake);
    wristUp = Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_UP_POSITION, operator), intake);
    wristMiddle = Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_MIDDLE_POSITION, operator), intake);
    getFuelUnstuck = Commands.runEnd(() -> {intake.spinIntake(0.6);}, () -> intake.stopIntake(), intake);
    intakeFuel = Commands.runEnd(() -> {intake.spinIntake(-1);}, () -> intake.stopIntake());
    shoot = Commands.runEnd(() -> shooter.shootWithDistance(1, photonVision.getRobotPose2d().get()), () -> shooter.stopShooterAndFeeder(), shooter);
    
    // turretShoot = Commands.runEnd(() -> shooter.shootWithoutPID(-0.17, -0.52, 1), () -> shooter.stopShooterAndFeeder(), shooter);
    turretShoot = Commands.runEnd(() -> shooter.shootWithSpeed(9.2, 9.4, 1), () -> shooter.stopShooterAndFeeder(), shooter);
    downtake = Commands.parallel(
      Commands.runEnd(() -> shooter.feed(-0.9), () -> shooter.feed(0), shooter),
      Commands.runEnd(() -> spindexer.SpindexerWithSpeed(-0.5), () -> spindexer.SpindexerWithSpeed(0), spindexer)
    );
    shootWhileMoving = Commands.runEnd(() -> shooter.shootWithDistance(1, AimAhead.getNextPose()), () -> shooter.stopShooterAndFeeder(), shooter);

    driveWithJoystick = new DriveWithJoystick(swere, driver, photonVision);
    moveWristWithJoystick = new MoveWristWithJoystick(intake, operator);
    fieldOrientedButton = new JoystickButton(driver, XboxController.Button.kA.value);
    turretToggleButton = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    turretFastModeToggleButton = new JoystickButton(operator, XboxController.Button.kA.value);
    toggleDriveAssistButton = new JoystickButton(driver, XboxController.Button.kB.value);
    shootButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    leftSideTurretTurnButton = new JoystickButton(operator, XboxController.Button.kX.value);
    rightSideTurretTurnButton = new JoystickButton(operator, XboxController.Button.kB.value);
    straightAheadTurretButton = new JoystickButton(operator, XboxController.Button.kY.value);
    // reverseIntakeButton = new JoystickButton(operator, XboxController.Button.kX.value);
    // intakeButton = new JoystickButton(operator, XboxController.Button.kA.value);
    downtakeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    toggleWristButton = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    wristDownButton = new POVButton(operator, 180);
    wristUpButton = new POVButton(operator, 0);
    wristMiddleButton = new POVButton(operator, 90);

    autoChooser = new SendableChooser<>();

    // NamedCommands.registerCommand("Shoot Command", Commands.run(() -> shooter.shootWithSpeed(5.98473400509, 11.7034798322, 1), shooter));
    NamedCommands.registerCommand("Shoot Command", Commands.runEnd(() -> shooter.shootWithDistance(1, photonVision.getHubCenterPose2d()), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(5));
    NamedCommands.registerCommand("Wrist Down", wristDown);
    NamedCommands.registerCommand("Wrist Up", wristUp);
    NamedCommands.registerCommand("Wrist Middle", Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_MIDDLE_POSITION, operator), intake).withTimeout(0.5));
    NamedCommands.registerCommand("Intake", intakeFuel);
    NamedCommands.registerCommand("Right Auto Turret Turn", Commands.run(() -> turret.turnTurret(120.11383056640625, operator), turret).withTimeout(0.75));
    NamedCommands.registerCommand("Outpost Turret Turn", Commands.run(() -> turret.turnTurret(63, operator), turret).withTimeout(0.75));
    NamedCommands.registerCommand("Right Outpost Start Turret Turn", Commands.run(() -> turret.turnTurret(109.23597635949386, operator), turret).withTimeout(0.75));

    autoChooser.addOption("Top Shooter Sys ID", Autos.topShooterSysID(shooter));
    autoChooser.addOption("Bottom Shooter Sys ID", Autos.bottomShooterSysID(shooter));
    autoChooser.addOption("Swerve Sys ID", Autos.swerveSysID(swere));
    autoChooser.addOption("Basic Center Auto", Autos.basicCenterAuto(swere, shooter,intake));
    autoChooser.addOption("Basic Left Auto", Autos.basicLeftAuto(swere, shooter));
    autoChooser.addOption("Basic Right Auto", Autos.basicRightAuto(swere, shooter));
    autoChooser.addOption("Right Outpost Shoot", Autos.rightOutpostShoot(swere, shooter));
    autoChooser.addOption("Basic Right Turret Auto", Autos.basicRightTurretAuto(swere, shooter, turret, photonVision, operator));
    swere.setDefaultCommand(driveWithJoystick);
    turret.setDefaultCommand(Commands.run(() -> turret.manualTurret(operator), turret));
    // turret.setDefaultCommand(turretToAngle);
    intake.setDefaultCommand(moveWristWithJoystick);
    spindexer.setDefaultCommand(spindexer.runSpindexer());

    SmartDashboard.putData(swere);
    SmartDashboard.putData(turret);
    SmartDashboard.putData(photonVision);
    SmartDashboard.putData(spindexer);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(intake);
    SmartDashboard.putData(autoChooser);

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
    toggleDriveAssistButton.onTrue(toggleDriveAssist); // 'B' button
    toggleWristButton.onTrue(toggleWristManual); // 'Left Joystick' button
    turretFastModeToggleButton.onTrue(toggleFastTurret);
    // shootButton.whileTrue(turretShoot);
    shootButton.whileTrue(shoot);
    // shootButton.whileTrue(shootWhileMoving);
    leftSideTurretTurnButton.onTrue(Commands.run(() -> turret.turnTurret(-90, operator), turret));
    rightSideTurretTurnButton.onTrue(Commands.run(() -> turret.turnTurret(90, operator), turret));
    straightAheadTurretButton.onTrue(Commands.run(() -> turret.turnTurret(0, operator), turret));
    wristUpButton.onTrue(wristUp);
    wristDownButton.onTrue(wristDown);
    wristMiddleButton.onTrue(wristMiddle);
    // wristUpButton.whileTrue(intakeFuel);
    // reverseIntakeButton.whileTrue(getFuelUnstuck);
    downtakeButton.whileTrue(downtake);
    // intakeButton.whileTrue(intakeFuel);
    operator.rightTriggerPressed().whileTrue(intakeFuel);
    operator.leftTriggerPressed().whileTrue(getFuelUnstuck);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
