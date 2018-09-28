package Team4450.Robot11;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import Team4450.Lib.NavX;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;

public class Devices
{
	  // Motor CAN ID/PWM port assignments (1=left-front, 2=left-rear, 3=right-front, 4=right-rear)
	  public static WPI_TalonSRX		LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon;
	  
	  public final static WPI_TalonSRX	intakeMotorL = new WPI_TalonSRX(5);
	  public final static WPI_TalonSRX	intakeMotorR = new WPI_TalonSRX(6);

	  public final static Talon			climbWinch = new Talon(0);			// PWM port 0.
	  public final static Talon			liftWinch = new Talon(1);			// PWM port 1.
	  public final static Servo			buddyBarDeployServo = new Servo(2);	// PWM port 2.
	  
	  public static DifferentialDrive	robotDrive;

	  public final static Joystick      utilityStick = new Joystick(2);	
	  public final static Joystick      leftStick = new Joystick(0);	
	  public final static Joystick		rightStick = new Joystick(1);	
	  public final static Joystick		launchPad = new Joystick(3);

	  public final static Compressor	compressor = new Compressor(0);	// Compressor class represents the PCM.
	  
	  public final static ValveDA		highLowValve = new ValveDA(0);
	  public final static ValveDA		grabberValve = new ValveDA(2);
	  public final static ValveDA		deployValve = new ValveDA(4);
	  public final static ValveDA		brakeValve = new ValveDA(6);
	  
	  public final static AnalogInput	pressureSensor = new AnalogInput(0);
	  
	  public final static PowerDistributionPanel	PDP = new PowerDistributionPanel();

	  public final static DriverStation				ds = DriverStation.getInstance();

	  public static NavX				navx;

	  // Wheel encoder is plugged into dio port 0 - orange=+5v blue=signal, dio port 1 black=gnd yellow=signal. 
	  public final static Encoder		wheelEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	  //public final static Encoder		wheelEncoder2 = new Encoder(2, 3, true, EncodingType.k4X);
	  public final static Encoder		winchEncoder = new Encoder(4, 5, true, EncodingType.k4X);
	  
	  public static SRXMagneticEncoderRelative	leftEncoder, rightEncoder;
	  
	  public static boolean				winchEncoderEnabled = true;
	  public static DigitalInput		winchSwitch = new DigitalInput(6);
	  
	  public final static SpeedControllerGroup grabberGroup = new SpeedControllerGroup(intakeMotorL, intakeMotorR);
	  
	  private static boolean			talonBrakeMode;
	  
	  // Create RobotDrive object for CAN Talon controllers.
	  
	  public static void InitializeCANTalonDrive()
	  {
		  Util.consoleLog();

		  LFCanTalon = new WPI_TalonSRX(1);
		  LRCanTalon = new WPI_TalonSRX(2);
		  RFCanTalon = new WPI_TalonSRX(3);
		  RRCanTalon = new WPI_TalonSRX(4);
		  
	      // Initialize CAN Talons and write status to log so we can verify
	      // all the Talons are connected.
	      InitializeCANTalon(LFCanTalon);
	      InitializeCANTalon(LRCanTalon);
	      InitializeCANTalon(RFCanTalon);
	      InitializeCANTalon(RRCanTalon);

	      InitializeCANTalon(intakeMotorL);
	      intakeMotorL.setNeutralMode(NeutralMode.Brake);
	      InitializeCANTalon(intakeMotorR);
	      intakeMotorR.setNeutralMode(NeutralMode.Brake);
	      
	      // Configure CAN Talons with correct inversions.
	      LFCanTalon.setInverted(true);
		  LRCanTalon.setInverted(true);
		  
		  RFCanTalon.setInverted(true);
		  RRCanTalon.setInverted(true);

	      // Turn on brake mode for drive CAN Talons.
	      SetCANTalonBrakeMode(true);
	      
	      // Setup the SpeedControllerGroups for the left and right set of motors.
	      SpeedControllerGroup LeftGroup = new SpeedControllerGroup(LFCanTalon, LRCanTalon);
		  SpeedControllerGroup RightGroup = new SpeedControllerGroup(RFCanTalon, RRCanTalon);
		  
		  rightEncoder = new SRXMagneticEncoderRelative(RFCanTalon, 5.8);
		  leftEncoder = new SRXMagneticEncoderRelative(LRCanTalon, 5.8);
		  leftEncoder.setScaleFactor(10);
		  leftEncoder.setInverted(true);
		  
		  robotDrive = new DifferentialDrive(LeftGroup, RightGroup);
	  }

	  // Initialize and Log status indication from CANTalon. If we see an exception
	  // or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	  public static void InitializeCANTalon(WPI_TalonSRX talon)
	  {
		  Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		  talon.clearStickyFaults(0); //0ms means no blocking.
		  //talon.enableControl();
		  //talon.changeControlMode(ControlMode.PercentOutput); //TODO Find PercentVbus
	  }
	  
	  // Set neutral behavior of drive CAN Talons. True = brake mode, false = coast mode.

	  public static void SetCANTalonBrakeMode(boolean brakeMode)
	  {
		  Util.consoleLog("brakes on=%b", brakeMode);
		  
		  SmartDashboard.putBoolean("Brakes", brakeMode);

		  talonBrakeMode = brakeMode;
		  
		  NeutralMode newMode;
		  
		  if (brakeMode) 
			  newMode = NeutralMode.Brake;
		  else 
			  newMode = NeutralMode.Coast;
		  
		  LFCanTalon.setNeutralMode(newMode);
		  LRCanTalon.setNeutralMode(newMode);
		  RFCanTalon.setNeutralMode(newMode);
		  RRCanTalon.setNeutralMode(newMode);
	  }
	  
	  public static boolean isBrakeMode()
	  {
		  return talonBrakeMode;
	  }
	  
	  // Set CAN Talon voltage ramp rate. Rate is number of seconds from zero to full output.
	  // zero disables.
	  
	  public static void SetCANTalonRampRate(double seconds)
	  {
		  Util.consoleLog("%.2f", seconds);
		  
		  LFCanTalon.configOpenloopRamp(seconds, 0);
		  LRCanTalon.configOpenloopRamp(seconds, 0);
		  RFCanTalon.configOpenloopRamp(seconds, 0);
		  RRCanTalon.configOpenloopRamp(seconds, 0);
	  }
	  
	  // Return voltage and current draw for each CAN Talon.
	  
	  public static String GetCANTalonStatus()
	  {
		  return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
				  LFCanTalon.getMotorOutputVoltage(), LFCanTalon.getOutputCurrent(),
				  LRCanTalon.getMotorOutputVoltage(), LRCanTalon.getOutputCurrent(),
				  RFCanTalon.getMotorOutputVoltage(), RFCanTalon.getOutputCurrent(),
				  RRCanTalon.getMotorOutputVoltage(), RRCanTalon.getOutputCurrent()
				  );
	  }

}
