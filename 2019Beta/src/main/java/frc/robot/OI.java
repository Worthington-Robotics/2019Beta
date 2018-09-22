/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.ForkUD;
import frc.robot.actions.Manipulate;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the actions and command groups that allow control of the robot.
 */
public class OI {
    public OI(){
        Button second2 = new JoystickButton(Constants.SECOND, 2);
        Button second3 = new JoystickButton(Constants.SECOND, 3);
        Button second4 = new JoystickButton(Constants.SECOND, 4);
        Button second5 = new JoystickButton(Constants.SECOND, 5);
        Button second8 = new JoystickButton(Constants.SECOND, 8);
        Button second9 = new JoystickButton(Constants.SECOND, 9);
        second2.whileHeld(Action.toCommand(new Manipulate(Manipulate.ShotPower.PickUp)));
        second3.whileHeld(Action.toCommand(new Manipulate(Manipulate.ShotPower.Shoot)));
        second4.whileHeld(Action.toCommand(new Manipulate(Manipulate.ShotPower.Drop)));
        second5.whileHeld(Action.toCommand(new Manipulate(Manipulate.ShotPower.SlowUp)));
        second8.whileHeld(Action.toCommand(new ForkUD(true)));
        second9.whileHeld(Action.toCommand(new ForkUD(false)));
    }
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // actions the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
