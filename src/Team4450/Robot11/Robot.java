/**
 * 2018 competition robot code.
 *
 * For Robot "Odyssey" built for FRC game "FIRST POWER UP".
*/

package Team4450.Robot11;

import java.util.Properties;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file.
 */

@SuppressWarnings("deprecation")
public class Robot extends SampleRobot 
{
  static final String  	PROGRAM_NAME = "RAC11ID-09.26.18-01";

  public Properties		robotProperties;
  
  public boolean		isClone = false, isComp = false;
  
  public RobotState		currentRobotState = RobotState.boot, lastRobotState = RobotState.boot;
    	
  DriverStation.Alliance	alliance;
  int                       location, matchNumber;
  String					eventName, gameMessage;
    
  Thread               	monitorBatteryThread, monitorPDPThread;
  MonitorCompressor		monitorCompressorThread;
  CameraFeed			cameraThread;
  
  Teleop 				teleOp;
  Autonomous 			autonomous;
  
  // Constructor.
  
  public Robot() //throws IOException
  {	
	// Set up our custom logger.
	 
	try
	{
		Util.CustomLogger.setup();
		
		// Catch any uncaught exceptions and record them in our log file. 
		
		Thread.setDefaultUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() 
		{
			public void uncaughtException(Thread t, Throwable e)
			{
		        Util.consoleLog("Uncaught exception from thread " + t);
		        Util.logException(e);
		    }

		});

		Util.consoleLog(PROGRAM_NAME);

    	Util.consoleLog("RobotLib=%s", LibraryVersion.version);
    }
    catch (Exception e) {Util.logException(e);}
  }
    
  // Initialization, called at class start up.
  
  public void robotInit()
  {
   	try
    {
   		Util.consoleLog();
        
        lastRobotState = RobotState.init;

   		LCD.clearAll();
   		LCD.printLine(1, "Mode: RobotInit");
      
   		// Read properties file from RoboRio "disk".
      
   		robotProperties = Util.readProperties();
      
   		// Is this the competition or clone robot?
   		
		if (robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;

   		SmartDashboard.putString("Program", PROGRAM_NAME);
   		
   		SmartDashboard.putBoolean("CompressorEnabled", Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault")));

   		// Reset PDB & PCM sticky faults.
      
   		Devices.PDP.clearStickyFaults();
   		Devices.compressor.clearAllPCMStickyFaults();
   		
   		// Configure motor controllers and RobotDrive.
   		
   		Devices.InitializeCANTalonDrive();
		
   		Devices.wheelEncoder.setReverseDirection(false);
   		//Devices.wheelEncoder2.setReverseDirection(false);

   		// Clone has reversed winch so we need invert the power so utility stick
		// operation remains the same.
		
		if (isClone) Devices.climbWinch.setInverted(true);
   		
		// Competition needs encoder reversed to read + as winch goes up.
		
   		if (isComp) Devices.winchEncoder.setReverseDirection(false);

   		Devices.robotDrive.stopMotor();
   		Devices.robotDrive.setSafetyEnabled(false);
   		Devices.robotDrive.setExpiration(0.1);
             
   		// Create NavX object here since must done before CameraFeed is created (don't remember why).
   		// Navx calibrates at power on and must complete before robot moves. Takes 12 seconds.

   		Devices.navx = NavX.getInstance(NavX.PortType.SPI);
   		
   		Devices.navx.dumpValuesToNetworkTables();

   		// Start the battery, compressor, PDP and camera feed monitoring Tasks.

   		monitorBatteryThread = MonitorBattery.getInstance();
   		monitorBatteryThread.start();

   		monitorCompressorThread = MonitorCompressor.getInstance(Devices.pressureSensor);
   		monitorCompressorThread.setDelay(1.0);
   		monitorCompressorThread.SetLowPressureAlarm(50);
   		monitorCompressorThread.start();
   		
   		//monitorPDPThread = MonitorPDP.getInstance(ds, PDP);
   		//monitorPDPThread.start();

   		// Start camera server using our class for usb cameras.
      
       	cameraThread = CameraFeed.getInstance(); 
       	cameraThread.start();
		
       	lastRobotState = currentRobotState;
       	
   		Util.consoleLog("end");
    }
    catch (Exception e) {Util.logException(e);}
  }
  
  // Called when robot is disabled.
  
  public void disabled()
  {
	  try
	  {
		  Util.consoleLog();
          
          lastRobotState = RobotState.disabled;

		  LCD.printLine(1, "Mode: Disabled");

		  // Reset driver station LEDs.

		  SmartDashboard.putBoolean("Disabled", true);
		  SmartDashboard.putBoolean("Auto Mode", false);
		  SmartDashboard.putBoolean("Teleop Mode", false);
		  SmartDashboard.putBoolean("FMS", Devices.ds.isFMSAttached());
		  SmartDashboard.putBoolean("AutoTarget", false);
		  SmartDashboard.putBoolean("TargetLocked", false);
		  SmartDashboard.putBoolean("Overload", false);
		  SmartDashboard.putNumber("AirPressure", 0);

		  Util.consoleLog("end");
	  }
	  catch (Exception e) {Util.logException(e);}
  }
  
  // Called at the start of Autonomous period.
  
  public void autonomous() 
  {
      try
      {
    	  Util.consoleLog();
          
          lastRobotState = RobotState.auto;

    	  LCD.clearAll();
    	  LCD.printLine(1, "Mode: Autonomous");
            
    	  SmartDashboard.putBoolean("Disabled", false);
    	  SmartDashboard.putBoolean("Auto Mode", true);
        
    	  getMatchInformation();
    	  
    	  // This code turns off the automatic compressor management if requested by DS.
    	  Devices.compressor.setClosedLoopControl(SmartDashboard.getBoolean("CompressorEnabled", true));

    	  // Reset persistent fault flags in control system modules.
    	  Devices.PDP.clearStickyFaults();
    	  Devices.compressor.clearAllPCMStickyFaults();
             
    	  // Start autonomous process contained in the Autonomous class.
        
    	  autonomous = new Autonomous(this);
        
    	  autonomous.execute();
    	  
    	  lastRobotState = currentRobotState;
      }
      catch (Exception e) {Util.logException(e);}
      
      finally
      {
      	  autonomous.dispose();

      	  SmartDashboard.putBoolean("Auto Mode", false);
      	  Util.consoleLog("end");
      }
  }

  // Called at the start of the teleop period.
  
  public void operatorControl() 
  {
      try
      {
    	  Util.consoleLog();
          
          currentRobotState = RobotState.teleop;

    	  LCD.clearAll();
      	  LCD.printLine(1, "Mode: Teleop");
            
      	  SmartDashboard.putBoolean("Disabled", false);
      	  SmartDashboard.putBoolean("Teleop Mode", true);
        
      	  getMatchInformation();
      	  
    	  // Reset persistent fault flags in control system modules.
          Devices.PDP.clearStickyFaults();
          Devices.compressor.clearAllPCMStickyFaults();

          // This code turns off the automatic compressor management if requested by DS.
          Devices.compressor.setClosedLoopControl(SmartDashboard.getBoolean("CompressorEnabled", true));
        
          // Start operator control process contained in the Teleop class.
        
          teleOp = new Teleop(this);
       
          teleOp.OperatorControl();
          
          lastRobotState = currentRobotState;
       }
       catch (Exception e) {Util.logException(e);} 
       
       finally
       {
           teleOp.dispose();
         	
           Util.consoleLog("end");
       }
  }
    
  public void test() 
  {
	  Util.consoleLog();
	  
	  currentRobotState = RobotState.test;
	  
	  lastRobotState = currentRobotState;
  }

  // Start usb camera server for single camera.
  
  public void StartUSBCameraServer(String cameraName, int device)
  {
	  Util.consoleLog("%s:%d", cameraName, device);

      CameraServer.getInstance().startAutomaticCapture(cameraName, device);
  }
  
  // Get information about the current match fromm the FMS or DS.
  
  public void getMatchInformation()
  {
  	  alliance = Devices.ds.getAlliance();
  	  location = Devices.ds.getLocation();
	  eventName = Devices.ds.getEventName();
	  matchNumber = Devices.ds.getMatchNumber();
	  gameMessage = Devices.ds.getGameSpecificMessage();
    
      Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, Devices.ds.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
  }
  
  public enum RobotState
  {
	  boot,
	  init,
	  disabled,
	  auto,
	  teleop,
	  test
  }
}
