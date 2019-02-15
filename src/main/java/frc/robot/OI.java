/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick joystick = new Joystick(0);
	JoystickButton btn1 = new JoystickButton(joystick, 1);
	JoystickButton btn2 = new JoystickButton(joystick, 2);
	JoystickButton btn3 = new JoystickButton(joystick, 3);
	JoystickButton btn4 = new JoystickButton(joystick, 4);
	JoystickButton btn5 = new JoystickButton(joystick, 5);
	JoystickButton btn6 = new JoystickButton(joystick, 6);
	JoystickButton btn7 = new JoystickButton(joystick, 7);
	JoystickButton btn8 = new JoystickButton(joystick, 8);
	JoystickButton btn9 = new JoystickButton(joystick, 9);
	JoystickButton btn10 = new JoystickButton(joystick, 10);
	JoystickButton btn11 = new JoystickButton(joystick, 11);
	JoystickButton btn12 = new JoystickButton(joystick, 12);
	
	public OI() {
		
		// btn3.whenPressed(new );
		// btn4.whenPressed(new );
		// btn5.whenPressed(new );
		// btn6.whenPressed(new );
		btn7.whenPressed(new MothToggle());
		btn9.whenPressed(new DriveForward(1));
		// btn9.whenPressed(new TurnLeft(135));
		btn11.whileHeld(new VisionHoldOnTarget());
		btn12.whenPressed(new VisionTurnToTarget());

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
	// commands the same as any other Button.

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
