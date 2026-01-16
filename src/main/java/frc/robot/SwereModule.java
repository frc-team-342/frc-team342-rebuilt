// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class SwereModule {
    private SparkMax driveMotor;
    private SparkMax rotateMotor;

    private SparkMaxConfig driveConfig;
    private SparkMaxConfig rotateConfig;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder rotateEncoder;
    private CANcoder rotateCANcoder;

    private SparkClosedLoopController driveController;
    private SparkClosedLoopController rotateController;

    private SwerveModuleState moduleState;

    private double encoderOffset;
    private double drivePositionConversion;
    private double driveVelocityConversion;

    private String label;

    public SwereModule(int driveID, int rotateID, int CANcoderId, boolean invertRotate, boolean invertDrive, double encoderOffset, double diameter, String label){
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        rotateMotor = new SparkMax(rotateID, MotorType.kBrushless);

        driveConfig = new SparkMaxConfig();
        rotateConfig = new SparkMaxConfig();

        driveConfig
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast)
            .inverted(invertDrive);
        rotateConfig
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast)
            .inverted(invertRotate);

        driveEncoder = driveMotor.getEncoder();
        rotateEncoder = rotateMotor.getEncoder();

        drivePositionConversion = (diameter * Math.PI) / (DriveConstants.DRIVE_GEAR_RATIO);
        driveVelocityConversion = drivePositionConversion / drivePositionConversion;

        driveConfig.encoder
            .positionConversionFactor(drivePositionConversion)
            .velocityConversionFactor(driveVelocityConversion);

        rotateConfig.encoder
            .positionConversionFactor(DriveConstants.ROTATE_POSITION_CONVERSION)
            .velocityConversionFactor(DriveConstants.ROTATE_VELOCITY_CONVERSION);

        driveController = driveMotor.getClosedLoopController();
        rotateController = rotateMotor.getClosedLoopController();

        driveConfig.closedLoop
            .p(DriveConstants.DRIVE_PIDF_VALUES[0])
            .i(DriveConstants.DRIVE_PIDF_VALUES[1])
            .d(DriveConstants.DRIVE_PIDF_VALUES[2]);
        rotateConfig.closedLoop
            .p(DriveConstants.ROTATE_PID_VALUES[0])
            .i(DriveConstants.ROTATE_PID_VALUES[1])
            .d(DriveConstants.ROTATE_PID_VALUES[2])
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(-Math.PI)
            .positionWrappingMaxInput(Math.PI);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotateMotor.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.encoderOffset = encoderOffset;
        this.label = label;

        rotateCANcoder = new CANcoder(CANcoderId);
        
        moduleState = new SwerveModuleState();
    }

    /**Returns the state of the swerve module (module velociy in m/s and module angle in radians) */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
                driveEncoder.getVelocity(), 
                new Rotation2d(rotateEncoder.getPosition())
            );
    }

    /**Sets the drive velocity and angle of the Swerve Module */
    public void setState(SwerveModuleState state){
        state.optimize(new Rotation2d(getRotateEncoderAngle()));

        driveController.setSetpoint(state.speedMetersPerSecond, ControlType.kVelocity);
        rotateController.setSetpoint(state.angle.getRadians(), ControlType.kPosition);
    }

    /**Returns the position of the swere module (distance traveled in meters and module angle in radians) */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(rotateEncoder.getPosition())
        );
    }

    /**@return distance robot drove in meters  */
    public double getDistance(){
        return driveEncoder.getPosition();
    }

    /**@return velocity of drive motor in m/s */
    public double getVelocity(){
        return driveEncoder.getVelocity();
    }

    /**@return angle of the CANcoder as a rotation 2d */
    public Rotation2d getRotation2d(){
        return new Rotation2d(rotateCANcoder.getPosition().getValueAsDouble());
    }

    /** @return encoder position in radians*/
    public double getRotateAngle(){
        return rotateEncoder.getPosition();
    }

    /**returns the ecoder position in radians between -pi and pi */
    public double getRotateEncoderAngle(){
        double angle = rotateEncoder.getPosition() % (2*Math.PI);
        return (angle > Math.PI) ? angle - (2.0*Math.PI) : angle;
    }

    /** @return rotate encoder position in radians */
    public double getRotateInRadian(){
        double angle = rotateCANcoder.getAbsolutePosition().getValueAsDouble() * DriveConstants.ROTATE_POSITION_CONVERSION;
        return (angle > Math.PI) ? angle - (2.0*Math.PI) : angle;
    }

    /**returns the raw offset of the module */
    public double getRawOffset(){
        return rotateCANcoder.getAbsolutePosition().getValueAsDouble();
    }
}
