// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;

public class CustomXboxController extends XboxController {

    /**
     * Constructs an instance of an Xbox Controller
     * @param port The driverstation port of the controller
    */
    public CustomXboxController(int port){
        super(port);
    }

    public double applyDeadband(double value){
        return MathUtil.applyDeadband(value, 0.15);
    }

    @Override
    public double getLeftX(){
        return applyDeadband(super.getLeftX());
    }

    @Override
    public double getLeftY(){
        return applyDeadband(super.getLeftY());
    }

    @Override
    public double getRightX(){
        return applyDeadband(super.getRightX());
    }

    @Override
    public double getRightY(){
        return applyDeadband(super.getRightY());
    }

    @Override
    public double getRawAxis(int axis){
        return applyDeadband(super.getRawAxis(axis));
    }

    /**@return Trigger instance that's true when the left trigger axis > 0.1 */
    public Trigger leftTriggerPressed(){
        return new Trigger(() -> super.getLeftTriggerAxis() > 0.1);
    }

    /**@return Trigger instance that's true when the right trigger axis > 0.1 */
    public Trigger rightTriggerPressed(){
        return new Trigger (() -> super.getRightTriggerAxis() > 0.1);
    }

    /**Makes the controller rumble at a given strength*/
    public Command rumble(double strength){
        return Commands.startEnd(
            () -> setRumble(RumbleType.kBothRumble, strength),
            () -> setRumble(RumbleType.kBothRumble, strength)
        );
    }

    /**Makes controller rumble at a given strength for a given amount of time*/
    public Command timedRumble(double time, double strength){
        return rumble(strength).withTimeout(time);
    }
}
