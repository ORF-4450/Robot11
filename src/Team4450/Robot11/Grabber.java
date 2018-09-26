package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grabber
{
	private final Robot	robot;
	private boolean		grabberOpen, grabberDeployed, intake, spit, autoIntake;
	private Thread		autoIntakeThread;
	
	public Grabber(Robot robot)
	{
		Util.consoleLog();
				
		this.robot = robot;
	  
		stopMotors();
		
		if (robot.isAutonomous()) retract();
		
		open();
	}

	public void dispose()
	{
		Util.consoleLog();
		
		if (autoIntakeThread != null) stopAutoIntake();
	}
		
	public void open()
	{
		Util.consoleLog();

		grabberOpen = true;
		
		SmartDashboard.putBoolean("Grabber", grabberOpen);

		if (robot.isClone)
			Devices.grabberValve.SetA();
		else
			Devices.grabberValve.SetB();

	}
	
	public void close()
	{
		Util.consoleLog();
		
		grabberOpen =  false;
		
		if (robot.isClone)
			Devices.grabberValve.SetB();
		else
			Devices.grabberValve.SetA();

		updateDS();
	}
	
	public void retract()
	{
		Util.consoleLog();
		
		grabberDeployed = false;

		if (robot.isClone)
			Devices.deployValve.SetA();
		else
			Devices.deployValve.SetB();

		updateDS();
	}
	
	public void deploy()
	{
		Util.consoleLog();
		
		grabberDeployed = true;
		
		if (robot.isClone)
			Devices.deployValve.SetB();
		else
			Devices.deployValve.SetA();

		updateDS();
	}
	
	public void motorsIn(double power)
	{
		Util.consoleLog("%.2f", power);
		
		intake = true;
		spit = false;

		Devices.grabberGroup.set(power);
		
		updateDS();
	}

	public void motorsOut(double power)
	{
		Util.consoleLog("%.2f", power);
		
		intake = false;
		spit = true;

		Devices.grabberGroup.set(-power);

		updateDS();
	}
	
	public void spit(double power)
	{
		Util.consoleLog("%.2f", power);
		
		motorsOut(power);
		
		Timer.delay(2);
		
		stopMotors();
	}
	
	public void stopMotors()
	{
		Util.consoleLog();
		
		intake = false;
		spit = false;

		Devices.grabberGroup.set(0);
		
		updateDS();
	}
	
	public boolean isOpen()
	{
		return grabberOpen;
	}
	
	public boolean isDeployed()
	{
		return grabberDeployed;
	}
	
	public boolean isIntake()
	{
		return intake;
	}
	
	public boolean isSpit()
	{
		return spit;
	}
	
	private void updateDS()
	{
		SmartDashboard.putBoolean("Grabber", grabberOpen);
		SmartDashboard.putBoolean("Deployed", grabberDeployed);
		SmartDashboard.putBoolean("Intake", intake);
		SmartDashboard.putBoolean("Spit", spit);
		SmartDashboard.putBoolean("AutoGrab", autoIntake);
	}
	
	public boolean isAutoIntakeRunning()
	{
		return autoIntake;
	}
	
	/**
	 * Start auto cube Intake thread.
	 */
	public void startAutoIntake()
	{
		Util.consoleLog();
		
		if (autoIntakeThread != null) return;

		autoIntakeThread = new AutoIntake();
		autoIntakeThread.start();
	}
	
	/**
	 * Stops auto Intake thread.
	 */
	public void stopAutoIntake()
	{
		Util.consoleLog();

		if (autoIntakeThread != null) autoIntakeThread.interrupt();
		
		autoIntakeThread = null;
	}

	//----------------------------------------
	// Automatic cube Intake thread.
	
	private class AutoIntake extends Thread
	{
		AutoIntake()
		{
			Util.consoleLog();
			
			this.setName("AutoCubeIntake");
	    }
		
	    public void run()
	    {
	    	double	stopCurrent, time = Util.timeStamp();
	    	
	    	Util.consoleLog();
	    	
	    	if (robot.isClone)
	    		stopCurrent = 17.0;	// 20.0;
	    	else
	    		stopCurrent = 15.0;
	    		
	    	try
	    	{
	    		autoIntake = true;
	    		
	    		updateDS();
	    		
	    		motorsIn(.50);
	    		sleep(250);
	    		
    	    	while (!isInterrupted() && robot.isEnabled() && 
    	    			!(Devices.intakeMotorL.getOutputCurrent() > stopCurrent ||
    	    			 Devices.intakeMotorR.getOutputCurrent() > stopCurrent))	// 15 // 5.0
    	    	{
    	            // We sleep since JS updates come from DS every 20ms or so. We wait 50ms so this thread
    	            // does not run at the same time as the teleop thread.
    	    		LCD.printLine(9, "cube motor current L=%.2f  R=%.2f", Devices.intakeMotorL.getOutputCurrent(),
    	    				Devices.intakeMotorR.getOutputCurrent());
    	    		
    	    		if (Util.getElaspedTime(time) > 7) autoIntakeThread.interrupt();
    	    		
    	            sleep(50);
    	    	}
    	    	
    	    	if (!interrupted() && robot.isEnabled()) Util.consoleLog("  Cube detected");
    	    	
    	    	sleep(500);
	    	}
	    	catch (InterruptedException e) { stopMotors(); }
	    	catch (Throwable e) { e.printStackTrace(Util.logPrintStream); }
	    	finally { stopMotors(); }
			
	    	autoIntake = false;
			autoIntakeThread = null;
			
			updateDS();
	    }
	}	// end of AutoIntake thread class.
}

