/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick joystick = new Joystick(0);
	public Joystick operator = new Joystick(1);

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

	JoystickButton opbtn1 = new JoystickButton(operator, 1);
	JoystickButton opbtn2 = new JoystickButton(operator, 2);
	JoystickButton opbtn3 = new JoystickButton(operator, 3);
	JoystickButton opbtn4 = new JoystickButton(operator, 4);
	JoystickButton opbtn5 = new JoystickButton(operator, 5);
	JoystickButton opbtn6 = new JoystickButton(operator, 6);
	JoystickButton opbtn7 = new JoystickButton(operator, 7);
	JoystickButton opbtn8 = new JoystickButton(operator, 8);
	JoystickButton opbtn9 = new JoystickButton(operator, 9);
	JoystickButton opbtn10 = new JoystickButton(operator, 10);
	JoystickButton opbtn11 = new JoystickButton(operator, 11);
	JoystickButton opbtn12 = new JoystickButton(operator, 12);
	
	public OI() {
		
		btn2.whileHeld(new VisionHatchPlacer());

		btn3.whenPressed(new ArmGoLow());
		btn4.whenPressed(new ArmGoRocket());
		btn5.whenPressed(new ArmGoCargo());

		btn6.whenPressed(new IntakeToggle());

		btn7.whenPressed(new DriverLowerGroup(false));
		btn8.whenPressed(new DriverRaiseGroup());

		btn9.whileHeld(new OuttakeBoth());
		btn10.whileHeld(new IntakeBoth(false));
		
		btn11.whenPressed(new MothToggle());
		
		btn12.whileHeld(new VisionHoldOnTarget());

		/*============================
		===== OPERATOR CONTROLS ======
		==============================*/

		opbtn2.whileHeld(new ArmZeroReset());

		opbtn3.whenPressed(new ArmGoLow());
		opbtn4.whenPressed(new ArmGoCargo());
		
		opbtn5.whenPressed(new DriverLowerGroup(false));
		opbtn6.whenPressed(new DriverRaiseGroup());

		opbtn7.whileHeld(new ArmManualDown());
		opbtn8.whileHeld(new ArmManualUp());

		opbtn9.whileHeld(new OuttakeBoth());
		opbtn10.whenPressed(new IntakeToggle());

		opbtn11.whenPressed(new ClimberSet(0.5,.3));  //expand
		opbtn11.whenPressed(new ClimbGroup());
		opbtn12.whileHeld(new ClimberSet(-0.5,0));
	}
}
