// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

/*
 * A SwerveModule refers to a singular wheel on the swerve chassis, commonly referred to
 * as a swerve module (hence the name of the class). There are 4 swerve modules on a typical swerve
 * chassis. Rather than writing the code for all 4 modules in a single subsystem, we create a class
 * to hold all the code for one swerve module. Then, we create 4 instances of SwerveModule in the
 * SwerveDrive subsystem, each corresponding to one of the four physical swerve module.
 */

/** Add your docs here. */

public class SwerveModule {

    /*
     * Each swerve module has two motors: a drive motor and a rotate motor.
     */
    private SparkFlex driveMotor;
    private SparkFlex rotateMotor;

    //Declare configs for each motor.
    private SparkFlexConfig driveConfig;
    private SparkFlexConfig rotateConfig;

    //Declare encoders for each motor.
    //These will always be relative encoders; absolute encoders are generally separate
    //from the motor controllers.
    private RelativeEncoder driveEncoder;
    private RelativeEncoder rotateEncoder;

    //Declare PID controllers for each motor.
    private SparkClosedLoopController driveController;
    private SparkClosedLoopController rotateController;
    
    //Declare the CANCoder.
    //These are the absolute encoders we have on the 2026 robot.
    private CANcoder rotateAbsoluteEncoder;
    private CANcoderConfiguration rotateAbsoluteEncoderConfig;

    //private PIDController rotatePID;

    /*
     * Declare a variable that will later be used to hold the
     * swerve module state. A SwerveModuleState consists of the speed
     * and angle of the module.
     */
    private SwerveModuleState swerveModuleState;

    //These two used to be used for something. I'm not entirely sure what, though.
    //They are currently not used at all.
    private double encoderOffset;
    private double driveVelConversion;

    //Diameter of the wheel.
    private double diameter;

    /*
     * The name of the module. This will correspond to
     * the physical location of the module. For example, the
     * front left module will have a label of "FL".
     */
    private String label;


    public SwerveModule (int driveID, int rotateID, int CANCoderPort, boolean invertRotate, boolean invertDrive, String label, double diameter){

        //Instantiate the motors and configs.
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        rotateMotor = new SparkFlex(rotateID, MotorType.kBrushless);

        driveConfig = new SparkFlexConfig();
        rotateConfig = new SparkFlexConfig();

        /*
         * The drive motors should always be in brake mode unless there's
         * something that requires them to be in coast mode to test.
         */
        driveConfig
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake)
            .inverted(invertDrive);

        /*
         * The rotate motors should also be typically in brake mode. However,
         * they can be in coast mode, as rotating generally carries less
         * momentum compared to driving.
         */
        rotateConfig
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast)
            .inverted(invertRotate);

        /** Get the encoders from the respective motors */
        driveEncoder = driveMotor.getEncoder();
        rotateEncoder = rotateMotor.getEncoder();

        /* Sets the Drive converstion (Posistion and Velocity)  factors  */
        driveConfig.encoder.positionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION); //POSITION
        driveConfig.encoder.velocityConversionFactor(0.00088); //VELOCITY

        /* Set the Rotate conversion (Posistion and Velocity) factors */
        rotateConfig.encoder.positionConversionFactor(DriveConstants.ROTATE_POSITION_CONVERSION); //POSITION
        rotateConfig.encoder.velocityConversionFactor(DriveConstants.ROTATE_VELOCITY_CONVERSION); //VELOCITY

        /** Get the PIDController from the respective motors */
        driveController = driveMotor.getClosedLoopController();
        rotateController = rotateMotor.getClosedLoopController();

        /* Sets the feedback sensor for each motor */
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rotateConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        /* Drive PID values */
        driveConfig.closedLoop.p(DriveConstants.DRIVE_PIDF_VALUES[0]);
        driveConfig.closedLoop.i(DriveConstants.DRIVE_PIDF_VALUES[1]);
        driveConfig.closedLoop.d(DriveConstants.DRIVE_PIDF_VALUES[2]);
        driveConfig.closedLoop.feedForward.sva(DriveConstants.DRIVE_SVA_VALUES[0], DriveConstants.DRIVE_SVA_VALUES[1], DriveConstants.DRIVE_SVA_VALUES[2]);

        /* Rotate PID wrapping */
        rotateConfig.closedLoop.positionWrappingEnabled(true);
        rotateConfig.closedLoop.positionWrappingMinInput(-Math.PI);
        rotateConfig.closedLoop.positionWrappingMaxInput(Math.PI);

         /* Rotate PID values */
        rotateConfig.closedLoop.p(DriveConstants.ROTATE_PID_VALUES[0], ClosedLoopSlot.kSlot0);
        rotateConfig.closedLoop.i(DriveConstants.ROTATE_PID_VALUES[1], ClosedLoopSlot.kSlot0);
        rotateConfig.closedLoop.d(DriveConstants.ROTATE_PID_VALUES[2], ClosedLoopSlot.kSlot0);
        // rotateConfig.closedLoop.pid(1.05, 0, 0.35, ClosedLoopSlot.kSlot1);

        /*Configures drive and rotate motors with their SparkFlex Config */

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotateMotor.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.label = label;
        this.diameter = diameter;

        /* Initializes the Analog Input and Analog Encoder. Analog Encoder acts as the absoulete encoder  */
        rotateAbsoluteEncoder = new CANcoder(CANCoderPort);

        rotateAbsoluteEncoderConfig = new CANcoderConfiguration();

        /*
         * We decided to use a switch statement to determine which offset to apply to the CANCoder.
         * There are other ways that may seem to be more intuitive or easier to understand. For
         * example, the CANCoder offset can be included as a parameter and given when instantiating the swerve module.
         */
        switch(CANCoderPort) {
            case DriveConstants.FRONT_LEFT_CANCODER_ID -> rotateAbsoluteEncoderConfig.MagnetSensor.MagnetOffset = DriveConstants.FL_OFFSET;
			case DriveConstants.FRONT_RIGHT_CANCODER_ID -> rotateAbsoluteEncoderConfig.MagnetSensor.MagnetOffset = DriveConstants.FR_OFFSET;
			case DriveConstants.BACK_LEFT_CANCODER_ID -> rotateAbsoluteEncoderConfig.MagnetSensor.MagnetOffset = DriveConstants.BL_OFFSET;
            case DriveConstants.BACK_RIGHT_CANCODER_ID -> rotateAbsoluteEncoderConfig.MagnetSensor.MagnetOffset = DriveConstants.BR_OFFSET;
        }

        /*
         * While these things can be set in Phoenix Tuner X, we want to set them
         * in the code anyways, just to be sure.
         */
        rotateAbsoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		rotateAbsoluteEncoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5);

        //Remember to apply the config.
		rotateAbsoluteEncoder.getConfigurator().apply(rotateAbsoluteEncoderConfig);

        //Instantiate the swerve module state as a new SwerveModuleState.
        //This will automatically give it a speed and rotation of 0.
        swerveModuleState = new SwerveModuleState();

        /*
         * I Accumulation only needs to be reset to 0 if an I value
         * is being used in the PID controller. Otherwise, this statement
         * is not necessary.
         */
        rotateController.setIAccum(0);

        // syncEncoders();
        // initSwerveState();
    }

    //This is used for swerve SysID. Look at the AdvantageKit swerve template for more information.
    public void runCharacterization(double output) {
        driveMotor.setVoltage(output);
        rotateController.setSetpoint(0, ControlType.kPosition);
    }

    /* Returns the distance robot has travlled in meters */
    public double getDistance() {
            return driveEncoder.getPosition();
    }

    /* Returns the Drive Encoder velocity meters/second */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**Returns the drive encoder velocity in rad/sec.
     * 
     * @return Velocity in rad/sec.
     */
    public double getDriveVelocityRad() {
        return getDriveVelocity() / (diameter / 2);
    }

    /* Returns the cancoder reading as a rotation2d */
    public Rotation2d canCoderRotation2d() {
        return new Rotation2d(rotateAbsoluteEncoder.getPosition().getValueAsDouble());
    }

    /* Returns the Angle of the wheels in Radians */
    public double getRotatePosition() {
        return rotateEncoder.getPosition();
    }

    /* Returns the position of the drive encoder */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /* Returns the Angle of the wheels in Radians */
    public double getRotateEncoderPosition(){
        
     double angle = rotateEncoder.getPosition();
     angle %= 2 * Math.PI;

        if (angle > Math.PI) {
            angle = angle - (2.0 * Math.PI);
        }

    return angle;

    }

    /* Sets the Rotation Encoder to the value of the analog offsets */
    public void syncEncoders(){
        rotateEncoder.setPosition(absoluteRotatePosition());
    }

    /* Uses the analog encoder to return the an angle within range in radians */
    public double absoluteRotatePosition() {

        double angle = rotateAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
        // if (angle > Math.PI) {
        //     angle = angle - (2 * Math.PI);
        // }

        return angle * 2 * Math.PI;

				// return (rotateAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()) * (2 * Math.PI);
    }

    /*
     * Initiates the swerve module state to a SwerveModuleState
     * with a speed of 0. The rotation of the swerve module is
     * set to the CANCoder reading.
     */
    public void initSwerveState() {
        // rotateController.setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        // rotateController.setSetpoint(0, ControlType.kPosition);

				setState(new SwerveModuleState(0.0, new Rotation2d(absoluteRotatePosition())));
    }

    /* Sets both motors to 0 */
    public void stop() {
       driveMotor.set(0);
       rotateMotor.set(0);
    }

    /* Spins the rotate motor at 20% speed */
    public void spinRotate(){
        rotateMotor.set(0.2);
    }

    /* Sets the voltage of the drive motor to the given voltage. */
    public void setDriveVoltage(double voltage) {
        driveMotor.set(voltage);
    }

    /* Returns the Label of specified module */
    public String printLabel() {
        return label;
    }
    
    /* Returns the current SwerveModuleState */
    public SwerveModuleState getState() {

        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotatePosition()));

    }

    /* Returns the setpoint of the drive PID controller */
    public double getDriveSetpoint(){
        return driveController.getSetpoint();
    }

    /* Returns the setpoint of the rotate PID controller */
    public double getRotateSetpoint(){
        return rotateController.getSetpoint();
    }

    /* Returns the drive velocity conversion */
    public double getVelocityConversion(){
        return driveVelConversion;
    }

    /* Returns the drive motor voltage */
    public double getDriveVoltage() {
        return driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    }
 
    /* Sets the refrence of drive and rotate motor */
    public void setState(SwerveModuleState state){

        state.optimize(new Rotation2d(getRotateEncoderPosition()));
        //state.cosineScale(new Rotation2d(getRotateEncoderPosition()));

        driveController.setSetpoint(state.speedMetersPerSecond /*/ driveVelConversion*/, ControlType.kVelocity);
        rotateController.setSetpoint(state.angle.getRadians(), ControlType.kPosition);
    }
}
