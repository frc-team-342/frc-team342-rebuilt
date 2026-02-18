// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.config.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkAnalogSensor;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;



/** Add your docs here. */

public class SwerveModule {

    private SparkFlex driveMotor;
    private SparkFlex rotateMotor;

    private SparkFlexConfig driveConfig;
    private SparkFlexConfig rotateConfig;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder rotateEncoder;

    private SparkClosedLoopController driveController;
    private SparkClosedLoopController rotateController;
    
    private CANcoder rotateAbsoluteEncoder;

    //private PIDController rotatePID;

    private SwerveModuleState swerveModuleState;
    
    private double encoderOffset;
    private double driveVelConversion;

    private String label;


    public SwerveModule (int driveID, int rotateID, int CANCoderPort, boolean invertRotate, boolean invertDrive, String label, double diameter){

        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        rotateMotor = new SparkFlex(rotateID, MotorType.kBrushless);

        driveConfig = new SparkFlexConfig();
        rotateConfig = new SparkFlexConfig();

        driveConfig
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast)
            .inverted(invertDrive);

        rotateConfig
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake)
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
        rotateConfig.closedLoop.p(DriveConstants.ROTATE_PID_VALUES[0]);
        rotateConfig.closedLoop.i(DriveConstants.ROTATE_PID_VALUES[1]);
        rotateConfig.closedLoop.d(DriveConstants.ROTATE_PID_VALUES[2]);

        /*Configures drive and rotate motors with there SparkFlex Config */

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotateMotor.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.label = label;

        /* Initializes the Analog Input and Analog Encoder. Analog Encoder acts as the absoulete encoder  */
        rotateAbsoluteEncoder = new CANcoder(CANCoderPort);

        swerveModuleState = new SwerveModuleState();

        syncEncoders();
    }

    /* Returns the distance robot has travlled in meters */
    public double getDistance() {
            return driveEncoder.getPosition();
    }

    /* Returns the Drive Encoder velocity meters/second */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /* Returns the cancoder reading as a rotation2d */
    public Rotation2d canCoderRotation2d() {
        return new Rotation2d(rotateAbsoluteEncoder.getPosition().getValueAsDouble());
    }

    /* Returns the Angle of the wheels in Radians */
    public double getRotatePosition() {
        return rotateEncoder.getPosition();
    }

    /* Returns the Angle of the wheels in Radians */
    public double getRotateEncoderPosition(){
        
     double angle = rotateEncoder.getPosition();
     angle %= 2 * Math.PI;

        if (angle > Math.PI) {
            angle = angle - 2.0 * Math.PI;
        }

    return angle;

    }

    /* Sets the Rotation Encoder to the value of the analog offsets */
    public void syncEncoders(){
        rotateEncoder.setPosition(absoluteRotatePosition());
    }

    /* Uses the analog encoder to return the an angle within range */
    public double absoluteRotatePosition() {

        double angle = rotateAbsoluteEncoder.getPosition().getValueAsDouble();
        if (angle > Math.PI) {
            angle = angle - 2 * Math.PI;
        }
         return angle;
    }

    /* Sets both motors too 0 */
    public void stop() {
       driveMotor.set(0);
       rotateMotor.set(0);
    }
    public void spinRotate(){
        rotateMotor.set(0.2);
    }

    /* Returns the Label of specified module */
    public String printLabel() {
        return label;
    }
    
    public SwerveModuleState getState() {

        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotatePosition()));

    }

    public double getDriveSetpoint(){
        return driveController.getSetpoint();
    }

    public double getRotateSetpoint(){
        return rotateController.getSetpoint();
    }

    public double getVelocityConversion(){
        return driveVelConversion;
    }
 
    /* Sets the refrence of drive and rotate motor */
    public void setState(SwerveModuleState state){

        state.optimize(new Rotation2d(getRotateEncoderPosition()));
        //state.cosineScale(new Rotation2d(getRotateEncoderPosition()));

        driveController.setSetpoint(state.speedMetersPerSecond /*/ driveVelConversion*/, ControlType.kVelocity);
        rotateController.setSetpoint(state.angle.getRadians(), ControlType.kPosition);
    }
}
