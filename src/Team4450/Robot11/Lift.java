package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift
{
	private final Robot			robot;
	// Only climb winch in use at the moment.
	private boolean				climbWinch = true, holdingPosition, holdingHeight;
	private boolean				buddyBarReleased, brakeEngaged;
	private final PIDController	liftPidController;
	
	public Lift(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;

		releaseBrake();
		servoExtendHalf();
		
		liftPidController = new PIDController(0.0, 0.0, 0.0, Devices.winchEncoder, Devices.climbWinch);
		
		Devices.winchEncoder.reset();
				
		updateDS();
	}

	public void dispose()
	{
		Util.consoleLog();
		
		liftPidController.disable();
		liftPidController.free();
	}
	
	public boolean isClimbWinchSelected()
	{
		Util.consoleLog();
		
		return climbWinch;
	}
	
	public void selectClimbWinch()
	{
		Util.consoleLog();

		setWinchPower(0);

		climbWinch = true;
		
		updateDS();
	}
	
	public void selectLiftWinch()
	{
		Util.consoleLog();

		climbWinch = false;
		
		updateDS();
	}
	
	public void servoExtendHalf()
	{
		Util.consoleLog();
		
		Devices.buddyBarDeployServo.setPosition(0.5);
		
		buddyBarReleased = true;
	}
	
	public void servoExtendFull()
	{
		Util.consoleLog();
		
		Devices.buddyBarDeployServo.setPosition(0.82);
		
		buddyBarReleased = true;
	}
	
	public void servoRetract()
	{
		Util.consoleLog();
		
		Devices.buddyBarDeployServo.setPosition(0.2);
		
		buddyBarReleased = false;
	}
	
	public boolean isBuddyBarReleased()
	{
		return buddyBarReleased;
	}
	
	public void releaseBrake()
	{
		Util.consoleLog();
		
		Devices.brakeValve.Open();
		
		brakeEngaged = false;
	}
	
	public void engageBrake()
	{
		Util.consoleLog();
		
		if (Devices.ds.getMatchTime() > 30) return;
		
		Devices.brakeValve.Close();
		
		brakeEngaged = true;
	}
	
	public boolean isBrakeEngaged()
	{
		return brakeEngaged;
	}
	
	public void updateDS()
	{
		SmartDashboard.putBoolean("Winch", climbWinch);
		SmartDashboard.putBoolean("TargetLocked", holdingHeight);
		SmartDashboard.putBoolean("Brake", brakeEngaged);
	}
	
	public void setWinchPower(double power)
	{
		if (isHoldingHeight()) return;
		
		if (climbWinch)
		{
			if (Devices.winchEncoderEnabled)
			{
//				if ((power > 0 && Devices.winchEncoder.get() < 10800) ||
//					(power < 0 && Devices.winchEncoder.get() > 0))
//					Devices.climbWinch.set(power);
//				else
//					Devices.climbWinch.set(0);
				
				// Competition robot has a limit switch and clone has hall effect sensor and the sensors
				// read in reverse so two sets of code.
				
				if (robot.isComp)
				{
					if ((power > 0 && Devices.winchEncoder.get() < 10800) ||	
						(power < 0 && !Devices.winchSwitch.get()))
						Devices.climbWinch.set(power);
					else
					{
						if (Devices.winchSwitch.get()) Devices.winchEncoder.reset();
						
						Devices.climbWinch.set(0);
					}
				}
				else
				{
					// Note that height limit is different because clone has different gear ratio in winch.
					if ((power > 0 && Devices.winchEncoder.get() < 14000) ||	// 10800
						(power < 0 && Devices.winchSwitch.get()))
						Devices.climbWinch.set(power);
					else
					{
						if (!Devices.winchSwitch.get()) Devices.winchEncoder.reset();
						
						Devices.climbWinch.set(0);
					}
				}
			}
			else
				Devices.climbWinch.set(power);
		}
		else
			Devices.liftWinch.set(power);
	}
	
	public void releaseBuddyBar()
	{
		Util.consoleLog();
		
		servoExtendHalf();
	}
	
//	public void releaseFoot()
//	{
//		Util.consoleLog();
//		
//		if ((robot.isClone && Devices.winchEncoder.get() > 8750) ||
//			(robot.isComp && Devices.winchEncoder.get() > 6500)) 	
//		{
//			Devices.footDeloyServo.setAngle(60);
//			footReleased = true;
//		}
//	}
	
	// Automatically move lift to specified encoder count and hold it there.
	// count < 0 turns pid controller off.
	
	public void setHeight(int count)
	{
		Util.consoleLog("%d", count);
		
		if (count >= 0)
		{
			if (isHoldingPosition()) holdPosition(0);
			
			// p,i,d values are a guess.
			// f value is the motor power to apply to move to encoder target count.
			// Setpoint is the target encoder count.
			// The idea is that the difference between the current encoder count and the
			// target count will apply power to bring the two counts together and stay there.
			liftPidController.setPID(0.0003, 0.00001, 0.0003, 0.0);
			//liftPidController.setPID(0.0003, 0.0, 0.0, 0.0);
			liftPidController.setOutputRange(-1, 1);
			liftPidController.setSetpoint(count);
			liftPidController.setPercentTolerance(1);	// % error.
			liftPidController.enable();
			holdingHeight = true;
		}
		else
		{
			liftPidController.disable();
			holdingHeight = false;
		}
		
		updateDS();
	}
	
	public boolean isHoldingHeight()
	{
		return holdingHeight;
	}
	
	public boolean isHoldingPosition()
	{
		return holdingPosition;
	}
	
	// Automatically hold lift position at specified power level.
	
	void holdPosition(double speed)
	{
		Util.consoleLog("%f", speed);
		
		if (speed != 0)
		{
			if (isHoldingHeight()) setHeight(-1);
			
			// p,i,d values are a guess.
			// f value is the base motor speed, which is where (power) we want to hold position.
			// Setpoint is current encoder count.
			// The idea is that any encoder motion will alter motor base speed to hold position.
			liftPidController.setPID(0.0003, 0.00001, 0.0003, speed);
			liftPidController.setSetpoint(Devices.winchEncoder.get());
			liftPidController.setPercentTolerance(1);	// % error.
			liftPidController.enable();
			holdingPosition = true;
		}
		else
		{
			liftPidController.disable();
			holdingPosition = false;
		}
	}
}

