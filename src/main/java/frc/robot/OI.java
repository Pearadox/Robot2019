/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.commands.ArmGoCargo;
import frc.robot.commands.ArmGoLow;
import frc.robot.commands.ArmGoRocket;
import frc.robot.commands.ArmManualDown;
import frc.robot.commands.ArmManualUp;
import frc.robot.commands.ArmZeroReset;
import frc.robot.commands.ClimbGroup;
import frc.robot.commands.ClimberSet;
import frc.robot.commands.DriverLowerGroup;
import frc.robot.commands.DriverRaiseGroup;
import frc.robot.commands.IntakeBoth;
import frc.robot.commands.IntakeToggle;
import frc.robot.commands.MothToggle;
import frc.robot.commands.OuttakeBoth;
import frc.robot.commands.VisionHatchPlacer;
import frc.robot.commands.VisionHoldOnTarget;
import frc.robot.utilities.ConditionalButton;
import frc.robot.utilities.ConditionalTrigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	enum ControllerMode {
		CARGO,
		HATCH
	}

	public final Controllers driveControllerType;
	private ControllerMode currentMode;

	public GenericHID driver = new Joystick(0);
	public Joystick operator = new Joystick(1);

	Trigger drvbtn1 = new JoystickButton(driver, 1);
	Trigger drvbtn2 = new JoystickButton(driver, 2);
	Trigger drvbtn3 = new JoystickButton(driver, 3);
	Trigger drvbtn4 = new JoystickButton(driver, 4);
	Trigger drvbtn5 = new JoystickButton(driver, 5);
	Trigger drvbtn6 = new JoystickButton(driver, 6);
	Trigger drvbtn7 = new JoystickButton(driver, 7);
	Trigger drvbtn8 = new JoystickButton(driver, 8);
	Trigger drvbtn9 = new JoystickButton(driver, 9);
	Trigger drvbtn10 = new JoystickButton(driver, 10);
	Trigger drvbtn11 = new JoystickButton(driver, 11);
	Trigger drvbtn12 = new JoystickButton(driver, 12);

	Trigger opbtn1 = new JoystickButton(operator, 1);
	Trigger opbtn2 = new JoystickButton(operator, 2);
	Trigger opbtn3 = new JoystickButton(operator, 3);
	Trigger opbtn4 = new JoystickButton(operator, 4);
	Trigger opbtn5 = new JoystickButton(operator, 5);
	Trigger opbtn6 = new JoystickButton(operator, 6);
	Trigger opbtn7 = new JoystickButton(operator, 7);
	Trigger opbtn8 = new JoystickButton(operator, 8);
	Trigger opbtn9 = new JoystickButton(operator, 9);
	Trigger opbtn10 = new JoystickButton(operator, 10);
	Trigger opbtn11 = new JoystickButton(operator, 11);
	Trigger opbtn12 = new JoystickButton(operator, 12);

	Trigger xboxDPad;
	
	public OI(Controllers driverType) {

		this.driveControllerType = driverType;

		if (driverType == Controllers.JOYSTICK) {
			driver = new Joystick(0);

			drvbtn1 = new JoystickButton(driver, 1);
			drvbtn2 = new JoystickButton(driver, 2);
			drvbtn3 = new JoystickButton(driver, 3);
			drvbtn4 = new JoystickButton(driver, 4);
			drvbtn5 = new JoystickButton(driver, 5);
			drvbtn6 = new JoystickButton(driver, 6);
			drvbtn7 = new JoystickButton(driver, 7);
			drvbtn8 = new JoystickButton(driver, 8);
			drvbtn9 = new JoystickButton(driver, 9);
			drvbtn10 = new JoystickButton(driver, 10);
			drvbtn11 = new JoystickButton(driver, 11);
			drvbtn12 = new JoystickButton(driver, 12);

			drvbtn2.whileActive(new VisionHatchPlacer());
			drvbtn3.whenActive(new ArmGoLow());
			drvbtn4.whenActive(new ArmGoRocket());
			drvbtn5.whenActive(new ArmGoCargo());
			drvbtn6.whenActive(new IntakeToggle());
			drvbtn7.whenActive(new DriverLowerGroup(false));
			drvbtn8.whenActive(new DriverRaiseGroup());
			drvbtn9.whileActive(new OuttakeBoth());
			drvbtn10.whileActive(new IntakeBoth(false));
			drvbtn11.whenActive(new MothToggle());
			drvbtn12.whileActive(new VisionHoldOnTarget());
		}
		if (driverType == Controllers.XBOX) {
			this.currentMode = ControllerMode.HATCH;
			driver = new XboxController(0);

			drvbtn1 = new JoystickButton(driver, 1); // A
			drvbtn2 = new JoystickButton(driver, 2); // B
			drvbtn3 = new JoystickButton(driver, 3); // X
			drvbtn4 = new JoystickButton(driver, 4); // Y
			drvbtn5 = new ConditionalButton(driver, 5, () -> currentMode == ControllerMode.HATCH); // LB
			drvbtn6 = new ConditionalButton(driver, 6, () -> currentMode == ControllerMode.HATCH); // RB
			drvbtn7 = new ConditionalButton(driver, 5, () -> currentMode == ControllerMode.CARGO); // LB
			drvbtn8 = new ConditionalButton(driver, 6, () -> currentMode == ControllerMode.CARGO); // RB
			drvbtn9 = new ConditionalTrigger(driver, 2, () -> currentMode == ControllerMode.HATCH); // LT
			drvbtn10 = new ConditionalTrigger(driver, 3, () -> currentMode == ControllerMode.HATCH); // RT
			drvbtn11 = new ConditionalTrigger(driver, 2, () -> currentMode == ControllerMode.CARGO); // LT
			drvbtn12 = new ConditionalTrigger(driver, 3, () -> currentMode == ControllerMode.CARGO); // RT
			xboxDPad = new Trigger() {
				@Override
				public boolean get() {
					return driver.getPOV() != -1;
				}
			};
			xboxDPad.whenActive(new ConditionalCommand(
				new InstantCommand(() -> currentMode = ControllerMode.CARGO),
				new InstantCommand(() -> currentMode = ControllerMode.HATCH)
				) {
			
				@Override
				protected boolean condition() {
					return currentMode == ControllerMode.HATCH;
				}
			}); // DPad (Any Button) - Switches Mode
			drvbtn1.whenActive(new ArmGoLow()); // A - Arm to low Height
			drvbtn2.whenActive(new ArmGoRocket()); // B - Arm to Rocket Height
			drvbtn3.whenActive(new IntakeToggle()); // X - Toggle Intake Raise/Lower
			drvbtn4.whenActive(new ArmGoCargo()); // Y - Arm to Cargo Height
			drvbtn5.whileActive(new VisionHatchPlacer()); // LB (Hatch Mode) - Auto Hatch Placer using vision
			drvbtn7.whileActive(new IntakeBoth(false)); // LB (Cargo Mode) - Intake Cargo
			drvbtn8.whenActive(new DriverRaiseGroup()); // RB (Cargo Mode) - Lower Arm/Intake
			drvbtn9.whileActive(new VisionHoldOnTarget()); // LT (Hatch Mode) - Limelight Thing?
			drvbtn10.whenActive(new MothToggle()); // RT (Hatch Mode) - Toggle Moth Open/Closed
			drvbtn11.whileActive(new OuttakeBoth()); // LT (Cargo Mode) - Outtake Cargo
			drvbtn12.whenActive(new DriverLowerGroup(false)); // RT (Cargo Mode) - Raise Arm/Intake
		}
		
		

		/*============================
		===== OPERATOR CONTROLS ======
		==============================*/

		opbtn2.whileActive(new ArmZeroReset());
		opbtn3.whenActive(new ArmGoLow());
		opbtn4.whenActive(new ArmGoCargo());
		opbtn5.whenActive(new DriverLowerGroup(false));
		opbtn6.whenActive(new DriverRaiseGroup());
		opbtn7.whileActive(new ArmManualDown());
		opbtn8.whileActive(new ArmManualUp());
		opbtn9.whileActive(new OuttakeBoth());
		opbtn10.whenActive(new IntakeToggle());
		opbtn11.whenActive(new ClimberSet(0.5,.3));  // open
		opbtn11.whenActive(new ClimbGroup());
		opbtn12.whileActive(new ClimberSet(-0.5,0));  // close
	}
}
