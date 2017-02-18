package org.oastem.frc.strong;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.sql.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import org.oastem.frc.C;
import org.oastem.frc.LogitechGamingPad;
import org.oastem.frc.control.TalonDriveSystem;
import org.oastem.frc.motion.MotionProfileExample;
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
	
	//Camera Values
	private double[] defaultValue = new double[0];
	private double[] centerY;
	private double[] centerX;
	private double centerXCoor = 0;
	
	//Sensors
	private LVMaxSonarEZUltrasonic sonicSensor;
	
	//Joystick 
	private LogitechGamingPad pad;
		
    //Joystick commands
	private boolean eStop1Pressed;
	private boolean reverseDirectionPressed;
	private boolean reverseDirectionToggle;
	private boolean conveyorPressed;
	private boolean conveyorToggle;
	private boolean upDPadToggle;
	private boolean downDPadToggle;
	
	//Joystick helpers
	private boolean stop;
	private boolean reverseOrNah;
	private boolean conveyorOrNah;
		
	// Network Table
	private NetworkTable table;
	
	//PDP
	private PowerDistributionPanel pdp;
		
	//Timer
	private Timer timer; 
	
	//Robot Preferences --> Used for testing
	private Preferences prefs;
	
	//Autonomous State
	//int autonomousCase = C.Auto.SELF_CORRECT_DRIVE;
	//final String case1Auto = "Straight";
	//final String case2Auto = "Left";
	//final String case3Auto = "Right";
	//SendableChooser<String> chooser = new SendableChooser<>();
	//String autoSelected;
	
	boolean reset = true; 

	private PrintWriter pw;
	
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
		winchMotor = new Talon(4);
		conveyorMotor = new Talon(5);
		
		//initialize Joystick 
		pad = new LogitechGamingPad(0);

		//initialize Camera Objects 
		server = CameraServer.getInstance();
		visionCamera = new AxisCamera("visionCamera", "10.40.79.88");
		
		
		//set Camera Values;
		visionCamera.setResolution(480, 360);
		server.startAutomaticCapture(visionCamera);
		
		//initialize Ultrasonic Sensor
		sonicSensor = new LVMaxSonarEZUltrasonic(C.Port.SONIC_SENSOR_INPUT_PORT);

		//initialize Timer
		timer = new Timer();

		//initialize Network Tables and Get Arrays from Contours Report
		table = NetworkTable.getTable("GRIP/myContoursReport");
		centerY = table.getNumberArray("centerY", defaultValue);
		centerX = table.getNumberArray("centerX", defaultValue);
		
		//initialize PDP
		pdp = new PowerDistributionPanel();
		pdp.clearStickyFaults();

		//set Joystick Toggles & Booleans
		reverseDirectionToggle = false;
		conveyorToggle = false;
		stop = false;
		reverseOrNah = false;
		conveyorOrNah = false;
		upDPadToggle = false;
		downDPadToggle = false;
		//conveyorOrNah = false;
		
		//Autonomous Chooser
		//chooser.addDefault("Case 1 (Default)", case1Auto);
		//chooser.addObject("Case 2: ", case2Auto);
		//chooser.addObject("Case 3: ", case3Auto);
		//SmartDashboard.putData("Auto choices", chooser);
		
		//initialize Preferences
		prefs = Preferences.getInstance();
		
		prefs.putDouble("Front Left Speed", 0.3);
		prefs.putDouble("Back Left Speed", 0.3);
		prefs.putDouble("Winch Motor Speed", 0);
		prefs.putDouble("IsForward", 1);
		
		/*DateFormat df = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		Calendar ca = Calendar.getInstance();
		try {
			//pw = new PrintWriter("debug" + df.format(ca) + ".txt", "UTF-8");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		*/
		
	}

	public void autonomousInit() {
		//autoSelected = chooser.getSelected();
		timer.start();
		resetEncoders();
		reset = true;
		
		talonDrive.getBackLeftDrive().changeControlMode(TalonControlMode.MotionMagic);
		talonDrive.getBackRightDrive().changeControlMode(TalonControlMode.MotionMagic);
		talonDrive.getBackLeftDrive().setProfile(0);
		talonDrive.getBackRightDrive().setProfile(0);
		
		/*
		
		backLeft.changeControlMode(TalonControlMode.MotionProfile);
		backRight.changeControlMode(TalonControlMode.MotionProfile);
		
		//if (autoSelected.equals(case1Auto)){
			leftProfile = new MotionProfileExample (backLeft, StraightCase1.getUpdatedPoints());
			rightProfile = new MotionProfileExample (backRight, StraightCase1.Points);
		//}
			/*
		else if (autoSelected.equals(case2Auto)){
			leftProfile = new MotionProfileExample (backLeft, LeftCase2.Points);
			rightProfile = new MotionProfileExample (backRight, RightCase2.Points);
		}
		else if (autoSelected.equals(case3Auto)){
			leftProfile = new MotionProfileExample (backLeft, LeftCase3.Points);
			rightProfile = new MotionProfileExample (backRight, RightCase3.Points);
		}
			leftProfile.startMotionProfile();
			rightProfile.startMotionProfile();
		 */
			
	}

	public void autonomousPeriodic() {
		double distance = 7 * 10.71; //in ft multiplied by gear ratio
		double circumference = 2*Math.PI*6;
		
		talonDrive.getBackLeftDrive().set(distance/circumference);
		talonDrive.getBackRightDrive().set(distance/circumference);
		printEncoderValues();
		/*
		leftProfile.control();
		rightProfile.control();
		
		CANTalon.SetValueMotionProfile setOutputLeft = leftProfile.getSetValue();
		CANTalon.SetValueMotionProfile setOutputRight = rightProfile.getSetValue();
		
		backLeft.set(setOutputLeft.value);
		backRight.set(setOutputRight.value);
		
		
		System.out.println(System.currentTimeMillis() + " BL: " + talonDrive.getBackLeftDrive().getOutputVoltage());
		System.out.println(System.currentTimeMillis() + " BR: " + talonDrive.getBackRightDrive().getOutputVoltage());
		System.out.println(System.currentTimeMillis() + " FL: " + talonDrive.getFrontLeftDrive().getOutputVoltage());
		System.out.println(System.currentTimeMillis() + " BL: " + talonDrive.getFrontRightDrive().getOutputVoltage());
		
		/**SmartDashboard.putNumber(System.currentTimeMillis() + " BL:",talonDrive.getBackLeftDrive().getOutputCurrent());
		SmartDashboard.putNumber(System.currentTimeMillis() + " BR:",talonDrive.getBackRightDrive().getOutputCurrent());
		SmartDashboard.putNumber(System.currentTimeMillis() + " FL:",talonDrive.getFrontLeftDrive().getOutputCurrent());
		SmartDashboard.putNumber(System.currentTimeMillis() + " FR:",talonDrive.getFrontRightDrive().getOutputCurrent());
		 */
		
		/*
		if (timer.get() < 0.6)
		{
			talonDrive.tankDrive(1*prefs.getDouble("IsForward", 1), 1*prefs.getDouble("IsForward", 1));
			if (timer.get() > 0.5)
			{
				if (reset){
					resetEncoders();
					reset = false;
				}
				SmartDashboard.putNumber("Rotations Left: ", talonDrive.getBackLeftDrive().getEncPosition());
				SmartDashboard.putNumber("Rotations Right: ", talonDrive.getBackRightDrive().getEncPosition());
			}
		}
		
		else 
			talonDrive.tankDrive(0, 0);
		*/
		
		
		/*centerY = table.getNumberArray("centerY", defaultValue);
		centerX = table.getNumberArray("centerX", defaultValue);

		switch (autonomousCase) {
		case C.Auto.DRIVE_STRAIGHT:
			if (timer.get() < 1.3)
				talonDrive.reverseTankDrive(C.Speed.STRAIGHT_DRIVE_SPEED, C.Speed.STRAIGHT_DRIVE_SPEED);
			else
				autonomousCase = C.Auto.MOMENTUM_STOP;
			break;

		case C.Auto.MOMENTUM_STOP:
			if (timer.get() < 4)
				talonDrive.tankDrive(C.Speed.STOP_SPEED, C.Speed.STOP_SPEED);
			else
				autonomousCase = C.Auto.TURN;
			break;

		case C.Auto.TURN:
			talonDrive.tankDrive(-C.Speed.TURN_SPEED, C.Speed.TURN_SPEED);

			if ( talonDrive.getAngle() < 65 && talonDrive.getAngle() > 55 &&  centerY.length == 2 && centerX.length == 2) {
				autonomousCase = C.Auto.STOP_WHEN_CENTERED;
			}
			break;

		case C.Auto.STOP_WHEN_CENTERED:
			centerXCoor = (centerX[0] + centerX[1]) / 2;
			if (centerXCoor >= C.Cam.HALF_RES_LEFT && centerXCoor <= C.Cam.HALF_RES_RIGHT) {
				talonDrive.tankDrive(C.Speed.STOP_SPEED, C.Speed.STOP_SPEED);
				autonomousCase = C.Auto.SELF_CORRECT_DRIVE;
			}
			break;

		case C.Auto.SELF_CORRECT_DRIVE:
			if (centerY.length == 2 && centerX.length == 2)
				centerXCoor = (centerX[0] + centerX[1]) / 2;
			SmartDashboard.putString("reached: ", "true");
			if (!(sonicSensor.getDistance() > C.Dist.MIN_DISTANCE
					&& sonicSensor.getDistance() < C.Dist.MAX_DISTANCE)) {
				if (centerXCoor >= C.Cam.HALF_RES_LEFT && centerXCoor <= C.Cam.HALF_RES_RIGHT)
					talonDrive.reverseTankDrive(C.Speed.SLOWER_DRIVE_SPEED, C.Speed.SLOWER_DRIVE_SPEED);
				else if (centerXCoor <= C.Cam.HALF_RES_LEFT)
					talonDrive.reverseTankDrive(C.Speed.SWERVE_DRIVE_SPEED, C.Speed.STOP_SPEED);
				else if (centerXCoor >= C.Cam.HALF_RES_RIGHT)
					talonDrive.reverseTankDrive(C.Speed.STOP_SPEED, C.Speed.SWERVE_DRIVE_SPEED);
				else if (centerY.length < 2 && centerX.length < 2)
					talonDrive.reverseTankDrive(C.Speed.SLOWER_DRIVE_SPEED, C.Speed.SLOWER_DRIVE_SPEED);
			} else {
				autonomousCase = C.Auto.STOP_WHEN_CLOSE_ENOUGH;
				SmartDashboard.putString("wtf: ", "false");
			}
			break;

		case C.Auto.STOP_WHEN_CLOSE_ENOUGH:
			if (sonicSensor.getDistance() > C.Dist.MIN_DISTANCE && sonicSensor.getDistance() < C.Dist.MAX_DISTANCE)
				talonDrive.tankDrive(C.Speed.STOP_SPEED, C.Speed.STOP_SPEED);
			break;
		}
		SmartDashboard.putNumber("Ultrasonic reading: ", sonicSensor.getDistance());
		SmartDashboard.putNumber("Ultrasonic raw reading; ", sonicSensor.getVoltage());
		*/
	}
	
	public void teleopInit(){
		talonDrive.resetGyro();
		resetEncoders();
		backLeft.changeControlMode(TalonControlMode.PercentVbus);
		backRight.changeControlMode(TalonControlMode.PercentVbus);
		timer.start();
		reset = true;
	}

	public void teleopPeriodic() {
		SmartDashboard.putNumber("Gyroscope value: ", talonDrive.getAngle());

		eStop1Pressed = pad.getBackButton();
		reverseDirectionPressed = pad.getAButton();
		conveyorPressed = pad.getYButton();
		
		
		if (eStop1Pressed)
			stop = true;
		
		if (reverseDirectionPressed && !reverseDirectionToggle)
		{
			reverseDirectionToggle = true;
			reverseOrNah = !reverseOrNah; 	
		}
		if (!reverseDirectionPressed)
			reverseDirectionToggle = false;

		if (reverseOrNah && !stop)
			talonDrive.tankDrive(0.5 * pad.getRightAnalogY() * (1 + pad.getRightTriggerValue()) , 0.5 * pad.getLeftAnalogY() * (1 + pad.getLeftTriggerValue()));
		
		else if (!reverseOrNah && !stop)
			talonDrive.reverseTankDrive(0.5 * pad.getRightAnalogY() * (1 + pad.getRightTriggerValue()) , 0.5 * pad.getLeftAnalogY() * (1+ pad.getLeftTriggerValue()));
		
		winchMotor.set(prefs.getDouble("Winch Motor Speed", 0));
		
		
		if (conveyorPressed && !conveyorToggle)
		{
			conveyorToggle = true;
			conveyorOrNah = !conveyorOrNah;
		}
		else if (!conveyorPressed)
			conveyorToggle = false; 
		
		if (conveyorOrNah)
			conveyorMotor.set(1);
		else if (!conveyorToggle)
			conveyorMotor.set(0);
		
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
		
		if (pad.getRightBumper())
			winchMotor.set(1);
		else if (pad.getLeftBumper())
			winchMotor.set(-1);
		else 
			winchMotor.set(0);
		
		SmartDashboard.putNumber("Left Speed: ", talonDrive.getBackLeftDrive().getSpeed());
		SmartDashboard.putNumber("Right Speed: ", talonDrive.getBackRightDrive().getSpeed());
		printEncoderValues();
		
		if (timer.get() > 5 && timer.get() < 5.1)
		{
			if (reset)
			{
				resetEncoders();
				reset = false;
			}
		}
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
		SmartDashboard.putNumber("Encoder Left Back: ", talonDrive.getBackLeftDrive().getEncPosition());
		SmartDashboard.putNumber("Encoder Right Back: ", talonDrive.getBackRightDrive().getEncPosition());
		SmartDashboard.putNumber("Encoder Left Front: ", talonDrive.getFrontLeftDrive().getEncPosition());
		SmartDashboard.putNumber("Encoder Right Front: ", talonDrive.getFrontRightDrive().getEncPosition());
	}
	
	public void testPeriodic() {
	}
}

