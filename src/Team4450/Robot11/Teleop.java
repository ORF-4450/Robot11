
package Team4450.Robot11;

import java.lang.Math;

import Team4450.Lib.*;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Teleop
{
	private final Robot 		robot;
	public  JoyStick			rightStick, leftStick, utilityStick;
	public  LaunchPad			launchPad;
	private boolean				autoTarget, invertDrive, altDriveMode;
	private Vision				vision;
	private GearBox				gearBox;
	private Lift				lift;
	private Grabber				grabber;

	// Constructor.

	Teleop(Robot robot)
	{
		Util.consoleLog();
		
		// Motor safety turned off during initialization.
		Devices.robotDrive.setSafetyEnabled(false);

		this.robot = robot;

		gearBox = new  GearBox(robot);
		
		lift = new Lift(robot);
		
		grabber = new Grabber(robot);
		
		vision = Vision.getInstance(robot);
	}

	// Free all objects that need it.

	void dispose()
	{
		Util.consoleLog();

		if (leftStick != null) leftStick.dispose();
		if (rightStick != null) rightStick.dispose();
		if (utilityStick != null) utilityStick.dispose();
		if (launchPad != null) launchPad.dispose();
		if (gearBox != null) gearBox.dispose();
		if (lift != null) lift.dispose();
		if (grabber != null) grabber.dispose();
	}

	void OperatorControl() throws Exception
	{
		double	rightY = 0, leftY = 0, utilX = 0, utilY = 0, rightX = 0, leftX = 0;
		double	gain = .05;
		boolean	steeringAssistMode = false;
		int		angle;

		// Motor safety turned off during initialization.
		Devices.robotDrive.setSafetyEnabled(false);

		Util.consoleLog();

		LCD.printLine(1, "Mode: OperatorControl");
		LCD.printLine(2, "All=%s, Start=%d, FMS=%b", robot.alliance.name(), robot.location, Devices.ds.isFMSAttached());

		// Configure LaunchPad and Joystick event handlers.

		launchPad = new LaunchPad(Devices.launchPad, LaunchPadControlIDs.BUTTON_BLUE, this);

		LaunchPadControl lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_LEFT_BACK);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;

		lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_LEFT_FRONT);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;

		lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_RIGHT);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;

		//Example on how to track button:
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_YELLOW);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_RED);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_RED_RIGHT);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_BLUE_RIGHT);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_GREEN);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_BLACK);

		launchPad.addLaunchPadEventListener(new LaunchPadListener());
		launchPad.Start();

		leftStick = new JoyStick(Devices.leftStick, "LeftStick", JoyStickButtonIDs.TRIGGER, this);
		//Example on how to track button:
		//leftStick.AddButton(JoyStickButtonIDs.BUTTON_NAME_HERE);
		leftStick.addJoyStickEventListener(new LeftStickListener());
		leftStick.Start();

		rightStick = new JoyStick(Devices.rightStick, "RightStick", JoyStickButtonIDs.TRIGGER, this);
		//Example on how to track button:
		//rightStick.AddButton(JoyStickButtonIDs.BUTTON_NAME_HERE);
		rightStick.AddButton(JoyStickButtonIDs.TOP_BACK);
		rightStick.AddButton(JoyStickButtonIDs.TOP_LEFT);
		rightStick.AddButton(JoyStickButtonIDs.TOP_RIGHT);
		rightStick.addJoyStickEventListener(new RightStickListener());
		rightStick.Start();

		utilityStick = new JoyStick(Devices.utilityStick, "UtilityStick", JoyStickButtonIDs.TRIGGER, this);
		//Example on how to track button:
		utilityStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
		utilityStick.AddButton(JoyStickButtonIDs.TOP_BACK);
		utilityStick.AddButton(JoyStickButtonIDs.TOP_LEFT);
		utilityStick.AddButton(JoyStickButtonIDs.TOP_RIGHT);
		utilityStick.addJoyStickEventListener(new UtilityStickListener());
		utilityStick.Start();

		// Set dead zone for smoother climber movement.
		utilityStick.deadZone(.20);
		
		// Invert driving joy sticks Y axis so + values mean forward. New for 2018
		// post season.
		leftStick.invertY(true);
		rightStick.invertY(true);

		// Set CAN Talon brake mode by rocker switch setting.
		// We do this here so that the Utility stick thread has time to read the initial state
		// of the rocker switch. Depends on lpcontrol being the last control defined for the 
		// launch pad as the one that controls brake mode.
		//if (robot.isComp) Devices.SetCANTalonBrakeMode(lpControl.latchedState);
		
		// Post season testing showed Anakin liked this setting, smoothing driving.
		// He also asked for brakes off in low gear, brakes on in high. See GearBox.
		// It controls brake setting.
		Devices.SetCANTalonRampRate(0.5);
		
		// Set Navx current yaw to 0.
		Devices.navx.resetYaw();

		// Reset encoder.
		//Devices.encoder.reset();
		Devices.wheelEncoder.reset();
		Devices.leftEncoder.reset();
		Devices.rightEncoder.reset();
		//Devices.LRCanTalon.setSensorPhase(false);
		//Devices.LRCanTalon.getSensorCollection().setQuadraturePosition(0, 0);
		//Devices.LRCanTalon.getSensorCollection().setPulseWidthPosition(1000, 0);
		
		// Motor safety turned on.
		Devices.robotDrive.setSafetyEnabled(true);

		// Driving loop runs until teleop is over.

		Util.consoleLog("enter driving loop");
		
		while (robot.isEnabled() && robot.isOperatorControl())
		{
			// Get joystick deflection and feed to robot drive object
			// using calls to our JoyStick class.

			rightY = stickLogCorrection(rightStick.GetY());	// fwd/back
			leftY = stickLogCorrection(leftStick.GetY());	// fwd/back

			rightX = stickLogCorrection(rightStick.GetX());	// left/right
			leftX = stickLogCorrection(leftStick.GetX());	// left/right

			utilY = utilityStick.GetY();

			LCD.printLine(3, "leftY=%.3f  rightY=%.3f  utilY=%.3f", leftStick.GetY(), rightStick.GetY(), utilY);
			LCD.printLine(4, "leftY=%.3f  rightY=%.3f  utilY=%.3f", leftY, rightY, utilY);
			LCD.printLine(5, "Wheel=%d  winch=%d  switch=%b", Devices.wheelEncoder.get(), 
					Devices.winchEncoder.get(), Devices.winchSwitch.get());
			LCD.printLine(6, "yaw=%.2f, total=%.2f, rate=%.2f, hdng=%.2f", Devices.navx.getYaw(), 
					Devices.navx.getTotalYaw(), Devices.navx.getYawRate(), Devices.navx.getHeading());
			LCD.printLine(7, "intake current L=%.2f  R=%.2f", Devices.intakeMotorL.getOutputCurrent(),
					 Devices.intakeMotorR.getOutputCurrent());
			LCD.printLine(8, "pressureV=%.2f  psi=%d", robot.monitorCompressorThread.getVoltage(), 
					robot.monitorCompressorThread.getPressure());
			LCD.printLine(9, "srx l=%d rot=%.2f - r=%d rot=%.2f d=%.2f", 
					Devices.leftEncoder.get(),
					Devices.leftEncoder.getRotations(),
					Devices.rightEncoder.get(),
					Devices.rightEncoder.getRotations(),
					Devices.rightEncoder.getDistance());

			// Set wheel motors.
			// Do not feed JS input to robotDrive if we are controlling the motors in automatic functions.

			//if (!autoTarget) robot.robotDrive.tankDrive(leftY, rightY);

			// Two drive modes, full tank and alternate. Switch on right stick trigger.

			if (!autoTarget) 
			{
				if (altDriveMode)
				{	// normal tank with straight drive assist when sticks within 10% of each other.
					if (isLeftRightEqual(leftY, rightY, 10) && Math.abs(rightY) > .50)
					{
						if (!steeringAssistMode) Devices.navx.resetYaw();

						// Angle is negative if robot veering left, positive if veering right when going forward.
						// It is opposite when going backward. Note that for this robot, - power means forward and
						// + power means backward.

						angle = (int) Devices.navx.getYaw();

						LCD.printLine(5, "angle=%d", angle);

						// Invert angle for backwards.

						if (rightY > 0) angle = -angle;

						//Util.consoleLog("angle=%d", angle);

						// Note we invert sign on the angle because we want the robot to turn in the opposite
						// direction than it is currently going to correct it. So a + angle says robot is veering
						// right so we set the turn value to - because - is a turn left which corrects our right
						// drift.
						
						// Update: The new curvatureDrive function expects the power to be + for forward motion.
						// Since our power value is - for forward, we do not invert the sign of the angle like
						// we did with previous drive functions. This code base should be updated to fix the
						// Y axis sign to be + for forward. This would make more sense and simplify understanding
						// the code and would match what curvatureDrive expects. Will wait on that until after
						// 2018 season. After fixing that, the angle would again need to be inverted.
						// Fixed for testing 4-23-18.

						Devices.robotDrive.curvatureDrive(rightY, -angle * gain, false);

						steeringAssistMode = true;
					}
					else
					{
						steeringAssistMode = false;
						Devices.robotDrive.tankDrive(leftY, rightY);		// Normal tank drive.
					}

					SmartDashboard.putBoolean("Overload", steeringAssistMode);
				}
				else
					Devices.robotDrive.tankDrive(leftY, rightY);		// Normal tank drive.
					// Don't forget to uncomment right stick trigger function in event handler.
					//Devices.robotDrive.curvatureDrive(rightY, rightX, rightStick.GetLatchedState(JoyStickButtonIDs.TRIGGER));
			}

			// Set winch power.
			
			lift.setWinchPower(utilY);
			
			// Update the robot heading indicator on the DS.

			SmartDashboard.putNumber("Gyro", Devices.navx.getHeadingInt());

			// End of driving loop.

			Timer.delay(.020);	// wait 20ms for update from driver station.
		}

		// End of teleop mode.

		// ensure we start next time in low gear.
		gearBox.lowSpeed();
		
		Util.consoleLog("end");
	}

	private boolean isLeftRightEqual(double left, double right, double percent)
	{
		if (Math.abs(left - right) <= (1 * (percent / 100))) return true;

		return false;
	}

	// Custom base logarithm.
	// Returns logarithm base of the value.

	private double baseLog(double base, double value)
	{
		return Math.log(value) / Math.log(base);
	}

	// Map joystick y value of 0.0 to 1.0 to the motor working power range of approx 0.5 to 1.0 using
	// logarithmic curve.

	private double stickLogCorrection(double joystickValue)
	{
		double base = Math.pow(2, 1/3) + Math.pow(2, 1/3);

		if (joystickValue > 0)
			joystickValue = baseLog(base, joystickValue + 1);
		else if (joystickValue < 0)
			joystickValue = -baseLog(base, -joystickValue + 1);

		return joystickValue;
	}

	// Handle LaunchPad control events.

	public class LaunchPadListener implements LaunchPadEventListener 
	{
		public void ButtonDown(LaunchPadEvent launchPadEvent) 
		{
			LaunchPadControl	control = launchPadEvent.control;

			Util.consoleLog("%s, latchedState=%b", control.id.name(),  control.latchedState);

			switch(control.id)
			{
				case BUTTON_RED:
					if (gearBox.isLowSpeed())
		    			gearBox.highSpeed();
		    		else
		    			gearBox.lowSpeed();
					
					break;
					
				case BUTTON_RED_RIGHT:
					if (grabber.isDeployed())
						grabber.retract();
					else
						grabber.deploy();
					
					break;
					
				case BUTTON_YELLOW:
					if (lift.isBrakeEngaged())
						lift.releaseBrake();
					else
						lift.engageBrake();

					break;
					
				case BUTTON_BLUE_RIGHT:
					// Automatic cube intake function.
					if (grabber.isAutoIntakeRunning())
						grabber.stopAutoIntake();
					else
						grabber.startAutoIntake();
					
					break;
					
//				case BUTTON_YELLOW:
//					if (lift.isHoldingHeight())
//						lift.setHeight(-1);
//					else
//						if (robot.isClone)
//							lift.setHeight(14000);
//						else
//							lift.setHeight(7900);
					
//					break;
					
				case BUTTON_GREEN:
					Devices.wheelEncoder.reset();
					Devices.leftEncoder.reset();
					Devices.rightEncoder.reset();
					break;
					
				default:
					break;
			}
		}

		public void ButtonUp(LaunchPadEvent launchPadEvent) 
		{
			//Util.consoleLog("%s, latchedState=%b", launchPadEvent.control.name(),  launchPadEvent.control.latchedState);
		}

		public void SwitchChange(LaunchPadEvent launchPadEvent) 
		{
			LaunchPadControl	control = launchPadEvent.control;

			Util.consoleLog("%s", control.id.name());

			switch(control.id)
			{
				// Set CAN Talon brake mode.
	    		case ROCKER_LEFT_BACK:
    				if (Devices.isBrakeMode())
    					Devices.SetCANTalonBrakeMode(false);	// coast
    				else
    					Devices.SetCANTalonBrakeMode(true);		// brake
    				
    				break;
    				
	    		case ROCKER_LEFT_FRONT:
					if (robot.cameraThread != null) robot.cameraThread.ChangeCamera();
					//invertDrive = !invertDrive;
	    			break;
	    			
	    		case ROCKER_RIGHT:
	    			if (control.latchedState)
	    			{
	    				Devices.winchEncoderEnabled = true;
	    				Devices.winchEncoder.reset();
	    			}
	    			else
	    				Devices.winchEncoderEnabled = false;
	    			
	    			break;
	
				default:
					break;
			}
		}
	}

	// Handle Right JoyStick Button events.

	private class RightStickListener implements JoyStickEventListener 
	{

		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				case TRIGGER:
					altDriveMode = !altDriveMode;
					break;
					
				case TOP_BACK:
					lift.servoRetract();
					break;
					
				case TOP_LEFT:
//					lift.servoExtendHalf();
//					break;
//		
					if (lift.isHoldingHeight())
						lift.setHeight(-1);
					else if (robot.isClone)
						lift.setHeight(14000);
					else
						lift.setHeight(7900);
				
					break;
					
				case TOP_RIGHT:
					lift.servoExtendHalf();
					break;

			//Example of Joystick Button case:
			/*
			case BUTTON_NAME_HERE:
				if (button.latchedState)
					DoOneThing();
				else
					DoOtherThing();
				break;
			 */
				default:
					break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.name());
		}
	}

	// Handle Left JoyStick Button events.

	private class LeftStickListener implements JoyStickEventListener 
	{
		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				case TRIGGER:
					if (gearBox.isLowSpeed())
	    				gearBox.highSpeed();
	    			else
	    				gearBox.lowSpeed();

					break;
					
				default:
					break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.name());
		}
	}

	// Handle Utility JoyStick Button events.

	private class UtilityStickListener implements JoyStickEventListener 
	{
		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				case TRIGGER:
					if (grabber.isOpen())
						grabber.close();
					else
						grabber.open();
					
					break;
					
				case TOP_MIDDLE:
					if (grabber.isSpit())
						grabber.stopMotors();
					else
						grabber.motorsOut(.50);
					
					break;
	
				case TOP_BACK:
					if (grabber.isIntake())
						grabber.stopMotors();
					else
						grabber.motorsIn(.50);
					
					break;
					
				case TOP_RIGHT:
					if (grabber.isAutoIntakeRunning())
						grabber.stopAutoIntake();
					else
						grabber.startAutoIntake();
					
					break;
					
				case TOP_LEFT:
					if (lift.isHoldingHeight())
						lift.setHeight(-1);
					else if (robot.isClone)
						lift.setHeight(900);
					else
						lift.setHeight(900);	// vault opening height.
					
					break;
	
				default:
					break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.id.name());
		}
	}
}
