// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final Shooter shooter;

  private final CustomXboxController driver;
  private final CustomXboxController operator;

  private final JoystickButton fieldOrientedButton;
  private final JoystickButton toggleWristButton;
  private final JoystickButton toggleJoystickControlButton;
  private final JoystickButton shootButton;
  private final JoystickButton toggleDriveAssistButton;
  private final JoystickButton downtakeButton;
  private final JoystickButton straightAheadTurretButton;
  private final JoystickButton leftSideTurretTurnButton;
  private final JoystickButton rightSideTurretTurnButton;
  private final POVButton wristDownButton;
  private final POVButton wristUpButton;
  private final POVButton wristMiddleButton;

  private final DriveWithJoystick driveWithJoystick;
  private final Command toggleFieldOriented;
  private final Command toggleDriveAssist;
  private final Command wristDown;
  private final Command wristUp;
  private final Command wristMiddle;
  private final Command intakeFuel;
  private final Command getFuelUnstuck;
  private final Command downtake;
  private final Command toggleWristManual;

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*
     * Creating instances of each subsystem. Make sure only one instance of each subsystem is created,
     * otherwise the code will throw an error. If you need to access one subsystem in another subsystem,
     * add it as a parameter so you don't create multiple instances of one subsystem.
     */

    /*
     * Creating two instances of CustomXboxController. Typically, there only need to be two
     * instances created in RobotContainer: driver and operator.
      */
    driver = new CustomXboxController(0);
    operator = new CustomXboxController(1);

    photonVision = new PhotonVision();
    swere = new SwerveDrive(photonVision);
    intake = new Intake();
    shooter = new Shooter(photonVision, operator);
    turret = new Turret(swere, photonVision, shooter);

    /*
     * Rather than creating commands for everything we want to do, we typically just make methods
     * inside the subsystems, then use the Commands class combined with lambdas to call those methods
     * inside of RobotContainer.
     */
    toggleFieldOriented = Commands.runOnce(() -> {swere.toggleFieldOriented();}, swere);
    toggleDriveAssist = Commands.runOnce(() -> {swere.toggleDriveAssist();}, swere);
    toggleWristManual = Commands.runOnce(() -> intake.toggleManual());
    wristDown = Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_DOWN_POSITION, operator), intake);
    wristUp = Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_UP_POSITION, operator), intake);
    wristMiddle = Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_MIDDLE_POSITION, operator), intake);
    getFuelUnstuck = Commands.runEnd(() -> {intake.spinIntake(0.6);}, () -> intake.stopIntake(), intake);
    intakeFuel = Commands.runEnd(() -> {intake.spinIntake(-1);}, () -> intake.stopIntake());
    downtake = Commands.parallel(
      Commands.runEnd(() -> shooter.feed(-0.9), () -> shooter.feed(0), shooter),
      Commands.runEnd(() -> shooter.SpindexerWithSpeed(-0.65), () -> shooter.SpindexerWithSpeed(0))
    );
    driveWithJoystick = new DriveWithJoystick(swere, driver);

    /*
     * Creating the buttons. Make sure that two buttons on the same controller
     * don't use the same value.
     */
    fieldOrientedButton = new JoystickButton(driver, XboxController.Button.kA.value);
    toggleDriveAssistButton = new JoystickButton(driver, XboxController.Button.kB.value);
    toggleJoystickControlButton = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    shootButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    leftSideTurretTurnButton = new JoystickButton(operator, XboxController.Button.kX.value);
    rightSideTurretTurnButton = new JoystickButton(operator, XboxController.Button.kB.value);
    straightAheadTurretButton = new JoystickButton(operator, XboxController.Button.kY.value);
    downtakeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    toggleWristButton = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    wristDownButton = new POVButton(operator, 180);
    wristUpButton = new POVButton(operator, 0);
    wristMiddleButton = new POVButton(operator, 90);

    /*
     * Creating an SendableChooser for autos. This needs to be done in order to
     * select autos, so don't forget it.
     */
    autoChooser = new SendableChooser<>();

    /*
     * When using PathPlanner, you need to register commands in order to use them in the
     * PathPlanner app.
     */
    // NamedCommands.registerCommand("Shoot Command", Commands.run(() -> shooter.shootWithSpeed(5.98473400509, 11.7034798322, 1), shooter));
    NamedCommands.registerCommand("Shoot Command", Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder(), shooter).withTimeout(5));
    NamedCommands.registerCommand("Wrist Down", Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_DOWN_POSITION, operator), intake).withTimeout(0.5));
    NamedCommands.registerCommand("Wrist Up", Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_UP_POSITION, operator), intake).withTimeout(0.5));
    NamedCommands.registerCommand("Wrist Middle", Commands.run(() -> intake.wristToPosition(IntakeConstants.WRIST_MIDDLE_POSITION, operator), intake).withTimeout(0.5));
    NamedCommands.registerCommand("Intake", Commands.runEnd(() -> {intake.spinIntake(-1);}, () -> intake.stopIntake()));
    NamedCommands.registerCommand("Right Auto Turret Turn", Commands.run(() -> turret.turnTurret(120.11383056640625), turret).withTimeout(0.75));
    NamedCommands.registerCommand("Outpost Turret Turn", Commands.run(() -> turret.turnTurret(63), turret).withTimeout(0.75));
    NamedCommands.registerCommand("Right Outpost Start Turret Turn", Commands.run(() -> turret.turnTurret(109.23597635949386), turret).withTimeout(0.75));

    /*
     * In order to run an auto, you must first add it to the auto chooser, so don't forget
     * to do this.
     */
    autoChooser.addOption("Drive SysId Quasistatic Forward", Commands.run(() -> swere.sysIdQuasistatic(SysIdRoutine.Direction.kForward), swere));
    autoChooser.addOption("Drive SysId Quasistatic Reverse", Commands.run(() -> swere.sysIdQuasistatic(SysIdRoutine.Direction.kReverse), swere));
    autoChooser.addOption("Drive SysId Dynamic Forward", Commands.run(() -> swere.sysIdDynamic(SysIdRoutine.Direction.kForward), swere));
    autoChooser.addOption("Drive SysId Dynamic Reverse", Commands.run(() -> swere.sysIdDynamic(SysIdRoutine.Direction.kReverse), swere));
    autoChooser.addOption("Top Shooter Sys ID", Autos.topShooterSysID(shooter));
    autoChooser.addOption("Bottom Shooter Sys ID", Autos.bottomShooterSysID(shooter));
    autoChooser.addOption("Swerve Sys ID", Autos.swerveSysID(swere));
    autoChooser.addOption("Basic Center Auto", Autos.basicCenterAuto(swere, shooter,intake));
    autoChooser.addOption("Basic Left Auto", Autos.basicLeftAuto(swere, shooter));
    autoChooser.addOption("Basic Right Auto", Autos.basicRightAuto(swere, shooter));
    autoChooser.addOption("Right Outpost Shoot", Autos.rightOutpostShoot(swere, shooter));
    autoChooser.addOption("Basic Right Turret Auto", Autos.basicRightTurretAuto(swere, shooter, turret, photonVision));
    autoChooser.addOption("Right Neutral Zone Auto", Autos.rightNeutralZoneAuto(swere, shooter, turret, photonVision, intake, operator));
    autoChooser.addOption("Straight Line Auto", Autos.straightLineAuto(swere));
    autoChooser.addOption("Basic Left Turret Auto", Autos.basicLeftTurretAuto(swere, shooter, turret, photonVision));
    autoChooser.addOption("Depot Auto", Autos.depotAuto(swere, shooter, turret, photonVision, intake, operator));
    
    /*
     * Setting default commands. These should be commands that you want to be run constantly,
     * as default commands are called every 20ms.
     */
    swere.setDefaultCommand(driveWithJoystick);
    turret.setDefaultCommand(Commands.run(() -> turret.trackLookAheadPose(photonVision.getHubCenterPose2d()), turret));

    /*
     * Putting data onto Elastic. This needs to be done in order for the initSendable data
     * to be shown on Elastic.
     */
    SmartDashboard.putData(swere);
    SmartDashboard.putData(turret);
    SmartDashboard.putData(photonVision);
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
    // resetGyroButton.onTrue(resetGyro);
    /*
     * Binding buttons to commands. Make sure you bind the correct buttons to the correct commands.
     * onTrue should be used for commands that you want to run once; whileTrue should be used for
     * commands that you want to run constantly while the button is being pressed.
     */
    fieldOrientedButton.onTrue(toggleFieldOriented); // 'A' button
    toggleDriveAssistButton.onTrue(toggleDriveAssist); // 'B' button
    toggleWristButton.onTrue(toggleWristManual); // 'Left Joystick' button
    toggleJoystickControlButton.onTrue(Commands.runOnce(() -> shooter.toggleJoystickControl()));
    shootButton.whileTrue(Commands.runEnd(() -> shooter.shootWithDistance(1, turret.getLookAheadPoses()[1]), () -> shooter.stopShooterAndFeeder()).alongWith(shooter.delayedSpinSpindexer()));
    // shootButton.whileTrue(Commands.runEnd(() -> shooter.shootWithSpeed(10.6, 10.4, 1.0), () -> shooter.stopShooterAndFeeder()).alongWith(shooter.delayedSpinSpindexer()));
    leftSideTurretTurnButton.onTrue(Commands.run(() -> turret.turnTurret(-90), turret));
    rightSideTurretTurnButton.onTrue(Commands.run(() -> turret.turnTurret(90), turret));
    straightAheadTurretButton.onTrue(Commands.run(() -> turret.turnTurret(0), turret));
    wristUpButton.onTrue(wristUp);
    wristDownButton.onTrue(wristDown);
    wristMiddleButton.onTrue(wristMiddle);
    downtakeButton.whileTrue(downtake);
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
    /*
     * Make sure to return autoChooser.getSelected() here. Otherwise,
     * the auto you choose will not run.
     */
    return autoChooser.getSelected();
  }
}
