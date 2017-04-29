package org.oastem.frc.steam;


import org.oastem.frc.C;
import org.oastem.frc.LogitechGamingPad;
import org.oastem.frc.control.TalonDriveSystem;
import org.oastem.frc.motion.LeftCase2;
import org.oastem.frc.motion.MotionProfileExample;
import org.oastem.frc.motion.RightCase2;
import org.oastem.frc.motion.StraightCase1;
import org.oastem.frc.sensor.LVMaxSonarEZUltrasonic;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	// Drive System
	private TalonDriveSystem talonDrive = TalonDriveSystem.getInstance();
		
	//Motors
	private CANTalon backLeft; //masters for mp
	private CANTalon backRight; //masters for mp 
	private Talon conveyorMotor;
	private Talon winchMotor;
	
	//Motion Profile Examples
	private MotionProfileExample leftProfile;
	private MotionProfileExample rightProfile; 
	
	//Camera Objects
	private CameraServer server;
	private AxisCamera visionCamera;
	private UsbCamera usbCamera;
	
	//Camera Values
	/*private double[] defaultValue = new double[0];
	private double[] centerY;
	private double[] centerX;
	private double centerXCoor = 0;
	
	//Sensors
	private LVMaxSonarEZUltrasonic sonicSensor;
	*/
	
	//Joystick 
	private LogitechGamingPad pad;
		
    //Joystick commands
	private boolean eStop1Pressed;
	//private boolean reverseDirectionPressed;
	//private boolean reverseDirectionToggle;
	private boolean conveyorPressed;
	private boolean conveyorToggle;
	private boolean upDPadToggle;
	private boolean downDPadToggle;
	//private boolean winchToggle;
	//private boolean winchPressed;
	private boolean slowPressed;
	private boolean slowToggle;
	
	//Joystick helpers
	private boolean stop;
	//private boolean reverseOrNah;
	private boolean conveyorOrNah;
	private boolean slowOrNah;
	//private boolean winchOrNah;
		
	// Network Table
	private NetworkTable table;
	
	//PDP
	private PowerDistributionPanel pdp;
		
	//Timer
	private Timer timer; 
	
	//Robot Preferences --> Used for testing
	private Preferences prefs;
	
	//Autonomous State
	int autonomousCase = C.Auto.SELF_CORRECT_DRIVE;
	final String case1Auto = "Straight";
	final String case2Auto = "Left";
	final String case3Auto = "Right";
	SendableChooser<String> chooser;
	String autoSelected;
	
	boolean start = false; 

	
	public Robot() {
        //initialize Drive System
		talonDrive.initializeTalonDrive(C.Port.FRONT_LEFT_CAN_DRIVE, C.Port.BACK_LEFT_CAN_DRIVE,
				C.Port.FRONT_RIGHT_CAN_DRIVE, C.Port.BACK_RIGHT_CAN_DRIVE, C.Drive.DRIVE_ENC_PULSE_PER_REV,
				C.Drive.DRIVE_WHEEL_DIAM);
		talonDrive.calibrateGyro();
		resetEncoders();
		
		//initialize master motors to use for motion profile 
		backLeft = talonDrive.getBackLeftDrive();
		backRight = talonDrive.getBackRightDrive(); 
		
		//initialize other motors
		winchMotor = new Talon(4);
		conveyorMotor = new Talon(5);
		
		//initialize Joystick 
		pad = new LogitechGamingPad(0);

		//initialize Camera Objects 
		server = CameraServer.getInstance();
		visionCamera = new AxisCamera("visionCamera", new String[] { "10.40.79.11", "axis-camera.local" });
		usbCamera = new UsbCamera("usbCamera",0);
		
		
		//set Camera Values;
		usbCamera.setResolution(80, 60);
		usbCamera.setFPS(5);
		visionCamera.setResolution(480, 360);
		server.startAutomaticCapture(visionCamera);
		server.startAutomaticCapture(usbCamera);
		
		//initialize Ultrasonic Sensor
		//sonicSensor = new LVMaxSonarEZUltrasonic(C.Port.SONIC_SENSOR_INPUT_PORT);

		//initialize Timer
		timer = new Timer();

		//initialize Network Tables and Get Arrays from Contours Report
		table = NetworkTable.getTable("GRIP/myContoursReport");
		//centerY = table.getNumberArray("centerY", defaultValue);
		//centerX = table.getNumberArray("centerX", defaultValue);
		
		//initialize PDP
		pdp = new PowerDistributionPanel();
		pdp.clearStickyFaults();

		//set Joystick Toggles & Booleans
		//reverseDirectionToggle = false;
		conveyorToggle = false;
		stop = false;
		//reverseOrNah = false;
		conveyorOrNah = false;
		upDPadToggle = false;
		downDPadToggle = false; 
		//winchOrNah = false;
		//winchToggle = false;
		slowOrNah = false;
		slowToggle = false;
		
		//Autonomous Chooser
		chooser = new SendableChooser<String>();
		chooser.addDefault("Straight Case",  case1Auto);
		chooser.addObject("Left Case", case2Auto);
		chooser.addObject("Right Case", case3Auto);
		chooser.addObject("No Auto", "No Auto");
		SmartDashboard.putData("Auto choices", chooser);
		
		//initialize Preferences
		prefs = Preferences.getInstance();
		
		prefs.putDouble("Left F-Gain", 2.2); 
		prefs.putDouble("Right F-Gain", 2);
		prefs.putDouble("Left P-Value", 1.1);
		prefs.putDouble("Right P-Value", 1);
		prefs.putDouble("Left I-Value", 0.001);
		prefs.putDouble("Right I-Value", 0.001);
		prefs.putDouble("Left D-Value", 0);
		prefs.putDouble("Right D-Value", 0);
		prefs.putInt("Left I-Zone", 100); //encoder counts
		prefs.putInt("Right I-Zone", 100); //encoder counts
		prefs.putInt("Left Tolerable Error", 20); //encoder counts
		prefs.putInt("Right Tolerable Error", 20); //encoder counts
		
	}

	public void autonomousInit() {
		autoSelected = (String)chooser.getSelected();
		timer.start();
		resetEncoders();
		start = false;
		
		backLeft.reset();
		backRight.reset();
		backLeft.enable();
		backRight.enable();
		
		resetEncoders();
		
		//backLeft.changeControlMode(TalonControlMode.Position);
		//backRight.changeControlMode(TalonControlMode.Position);
		
		backLeft.changeControlMode(TalonControlMode.MotionProfile);
		backRight.changeControlMode(TalonControlMode.MotionProfile);
		
		backLeft.setF(prefs.getDouble("Left F-Gain", 0));
		backRight.setF(prefs.getDouble("Right F-Gain", 0));
		backLeft.setP(prefs.getDouble("Left P-Value", 0));
		backRight.setP(prefs.getDouble("Right P-Value", 0));
		backLeft.setI(prefs.getDouble("Left I-Value", 0));
		backRight.setI(prefs.getDouble("Right I-Value", 0));
		backLeft.setD(prefs.getDouble("Left D-Value", 0));
		backRight.setD(prefs.getDouble("Right D-Value", 0));
		backLeft.setIZone(prefs.getInt("Left I-Zone", 0));
		backRight.setIZone(prefs.getInt("Right I-Zone", 0));
		backLeft.setAllowableClosedLoopErr(prefs.getInt("Left Tolerable Error", 0));
		backRight.setAllowableClosedLoopErr(prefs.getInt("Right Tolerable Error", 0));
		backLeft.setVoltageRampRate(0);
		backRight.setVoltageRampRate(0);
		
		if (autoSelected.equals("No Auto"))
		{
			backLeft.changeControlMode(TalonControlMode.PercentVbus);
			backRight.changeControlMode(TalonControlMode.PercentVbus);
			
		}
		else {
			if (autoSelected.equals(case1Auto)){
				leftProfile = new MotionProfileExample (backLeft, StraightCase1.Points);
				rightProfile = new MotionProfileExample (backRight, StraightCase1.getUpdatedPoints());
				}
			else if (autoSelected.equals(case2Auto)){
				leftProfile = new MotionProfileExample (backLeft, RightCase2.Points); 
				rightProfile = new MotionProfileExample (backRight, LeftCase2.getUpdatedPoints()); 
				}
			else if (autoSelected.equals(case3Auto)){
				leftProfile = new MotionProfileExample (backLeft, LeftCase2.Points);
				rightProfile = new MotionProfileExample (backRight, RightCase2.getUpdatedPoints());
				}
			
			rightProfile.startMotionProfile();
			leftProfile.startMotionProfile();
		
			rightProfile.reset();
			leftProfile.reset();
		}
		
		timer.start();
		
		//backLeft.set(30);
		//backRight.set(30);
	}

	public void autonomousPeriodic() {
		if (!autoSelected.equals("No Auto"))
		{
			if (timer.get() > 0.2){
				rightProfile.startMotionProfile();
				leftProfile.startMotionProfile();
				start = true; 
			}
		
			if (start){
				rightProfile.control();
				leftProfile.control();
		
				CANTalon.SetValueMotionProfile setOutputRight = rightProfile.getSetValue();
				CANTalon.SetValueMotionProfile setOutputLeft = leftProfile.getSetValue();
		
				backRight.set(setOutputRight.value);
				backLeft.set(setOutputLeft.value);
			}
		}
		printEncoderValues();
	}
	
	
	public void teleopInit(){
		talonDrive.resetGyro();
		resetEncoders();
		
		backLeft.changeControlMode(TalonControlMode.PercentVbus);
		backRight.changeControlMode(TalonControlMode.PercentVbus);
		
		backLeft.setVoltageRampRate(0);
		backRight.setVoltageRampRate(0);
		//reverseOrNah = false;
		slowOrNah = false;
	}

	public void teleopPeriodic() {
		SmartDashboard.putNumber("Gyroscope value: ", talonDrive.getAngle());

		eStop1Pressed = pad.getBackButton();
		//reverseDirectionPressed = pad.getAButton();
		conveyorPressed = pad.getYButton();
		//winchPressed = pad.getXButton();
		slowPressed = pad.getBButton();
		
		
		if (eStop1Pressed)
			stop = true;
		/*
		//REVERSE TOGGLE
		if (reverseDirectionPressed && !reverseDirectionToggle)
		{
			reverseDirectionToggle = true;
			reverseOrNah = !reverseOrNah; 	
		}
		if (!reverseDirectionPressed)
			reverseDirectionToggle = false;
		*/
		
		//SLOW TOGGLE
		if (slowPressed && !slowToggle)
		{
			slowToggle = true;
			slowOrNah = !slowOrNah;
		}
		else if (!slowPressed)
					slowToggle = false; 
		
		if ( !stop && /*reverseOrNah &&*/ !slowOrNah)
			talonDrive.tankDrive(-(0.5 * pad.getLeftAnalogY() * (1 + pad.getLeftTriggerValue())), (0.5 * pad.getRightAnalogY() * (1 + pad.getRightTriggerValue())) );
		
		//else if (!stop && !reverseOrNah && !slowOrNah)
			//talonDrive.reverseTankDrive(0.5 * pad.getRightAnalogY() * (1 + pad.getRightTriggerValue()) , 0.5 * pad.getLeftAnalogY() * (1+ pad.getLeftTriggerValue()));
		
		else if (!stop && /*reverseOrNah &&*/ slowOrNah)
			talonDrive.tankDrive(-0.25 * pad.getLeftAnalogY() , 0.25 * pad.getRightAnalogY());
		
		//else if (!stop && !reverseOrNah && slowOrNah)
		//talonDrive.reverseTankDrive(0.25 * pad.getRightAnalogY() , 0.25 * pad.getLeftAnalogY());
		
		
		//CONVEYOR TOGGLE
		if (conveyorPressed && !conveyorToggle)
		{
			conveyorToggle = true;        
			conveyorOrNah = !conveyorOrNah;
		}
		else if (!conveyorPressed)
			conveyorToggle = false; 
		
		if (conveyorOrNah)
			conveyorMotor.set(-1);
		else if (!conveyorOrNah)
			conveyorMotor.set(0);
		
		/*//WINCH TOGGLE
		if (winchPressed && !winchToggle)
		{
			winchToggle = true;
			winchOrNah = !winchOrNah;
		}
		else if (!winchPressed)
			winchToggle = false; 
		
		if (winchOrNah)
			winchMotor.set(-1);
		else if (!winchOrNah)
			winchMotor.set(0);
		*/
		
		//WINCH CONTINUOUS 
		if (pad.getRightBumper())
			winchMotor.set(-1);
		else if (pad.getLeftBumper())
			winchMotor.set(1);
		else 
			winchMotor.set(0);
		
		//TEST
		if (pad.checkDPad(0) && !upDPadToggle)
		{
			upDPadToggle = true;
			talonDrive.resetGyro();
		}
		if (!pad.checkDPad(0))
			upDPadToggle = false;
		
		if (pad.checkDPad(0))
		{
			if (talonDrive.getAngle() > C.Speed.GYRO_ANGLE_LIMIT)
				talonDrive.tankDrive(C.Speed.GYRO_SLOWER_SPEED, C.Speed.STRAIGHT_DRIVE_SPEED);
			else if (talonDrive.getAngle() < C.Speed.GYRO_ANGLE_LIMIT)
				talonDrive.tankDrive(C.Speed.STRAIGHT_DRIVE_SPEED, C.Speed.GYRO_SLOWER_SPEED);
			else
				talonDrive.tankDrive(C.Speed.STRAIGHT_DRIVE_SPEED, C.Speed.STRAIGHT_DRIVE_SPEED);
		}
		
		if (pad.checkDPad(4) && !downDPadToggle)
		{
			downDPadToggle = true;
			talonDrive.resetGyro();
		}
		if (!pad.checkDPad(4))
			downDPadToggle = false;
		
		if (pad.checkDPad(4))
		{
			if (talonDrive.getAngle() > C.Speed.GYRO_ANGLE_LIMIT)
				talonDrive.reverseTankDrive(C.Speed.GYRO_SLOWER_SPEED, C.Speed.STRAIGHT_DRIVE_SPEED);
			else if (talonDrive.getAngle() < C.Speed.GYRO_ANGLE_LIMIT)
				talonDrive.reverseTankDrive(C.Speed.STRAIGHT_DRIVE_SPEED, C.Speed.GYRO_SLOWER_SPEED);
			else
				talonDrive.reverseTankDrive(C.Speed.STRAIGHT_DRIVE_SPEED, C.Speed.STRAIGHT_DRIVE_SPEED);
		}
		
		
		printEncoderValues();
		
	}

	public void resetEncoders()
	{
		talonDrive.getBackLeftDrive().setEncPosition(0);
		talonDrive.getFrontLeftDrive().setEncPosition(0);
		talonDrive.getFrontRightDrive().setEncPosition(0);
		talonDrive.getBackRightDrive().setEncPosition(0);
		
	} 
	
	private double scaleTrigger(double trigger) {
		return Math.min(1.0, 1.0 - 0.9 * trigger);
	}

	public double predictDistance(double pixels) {
		// y = -1.542693 + (2487.625 - -1.542693)/(1 + (x/1.294396)^0.9814597)
		double num = 2487.625 + 1.542693;
		double den = 1 + Math.pow((pixels / 1.294396), 0.9814597);
		return (num / den - 1.542693);
	}
	
	public void printEncoderValues()
	{
		double leftEnc = talonDrive.getBackLeftDrive().getEncPosition();
		double rightEnc = talonDrive.getBackRightDrive().getEncPosition();
		SmartDashboard.putNumber("Encoder Left Back: ", leftEnc);
		SmartDashboard.putNumber("Encoder Right Back: ", rightEnc);
		SmartDashboard.putNumber("Encoder Left Front: ", talonDrive.getFrontLeftDrive().getEncPosition());
		SmartDashboard.putNumber("Encoder Right Front: ", talonDrive.getFrontRightDrive().getEncPosition());
		//SmartDashboard.putNumber("Distance: ", sonicSensor.getDistance());
		SmartDashboard.putNumber("Left Encoder Distance: ", leftEnc/80/10.71*6*Math.PI);
		SmartDashboard.putNumber("Right Encoder Distance: ", rightEnc/80/10.71*6*Math.PI);
		SmartDashboard.putNumber("Right Error: ", backRight.getClosedLoopError()); //encoder counts
		SmartDashboard.putNumber("Left Error: ", backLeft.getClosedLoopError()); //encoder counts
		SmartDashboard.putNumber("Gyroscope Value ",talonDrive.getAngle());
	}
	
	public void testPeriodic() {
	}
}

