package org.oastem.frc.strong;

import org.oastem.frc.C;
import org.oastem.frc.LogitechGamingPad;
import org.oastem.frc.control.TalonDriveSystem;
import org.oastem.frc.sensor.LVMaxSonarEZUltrasonic;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	// Drive System
	private TalonDriveSystem talonDrive = TalonDriveSystem.getInstance();
	
	//Motors
	private Talon conveyorMotor;
	private Talon winchMotor;
	
	//Camera Objects
	private CameraServer server;
	private UsbCamera camera;
	private UsbCamera camera2;
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
	private double slowTrigger;
	private boolean eStop1Pressed;
	private boolean reverseDirectionPressed;
	private boolean reverseDirectionToggle;
	private boolean sprintLeft;
	private boolean sprintRight;
	private boolean conveyorPressed;
	private boolean conveyorToggle;
	private double winch;
	
	// Network Table
	private NetworkTable table;
	
	//PDP
	private PowerDistributionPanel pdp;
	
	//Timer
	private Timer timer; 
	
	public Robot() {
		//initialize drive system
		talonDrive.initializeTalonDrive(C.Port.FRONT_LEFT_CAN_DRIVE, C.Port.BACK_LEFT_CAN_DRIVE,
				C.Port.FRONT_RIGHT_CAN_DRIVE, C.Port.BACK_RIGHT_CAN_DRIVE, C.Drive.DRIVE_ENC_CODE_PER_REV,
				C.Drive.DRIVE_WHEEL_DIAM);
		talonDrive.calibrateGyro();
		resetEncoders();
		
		//initialize joystick 
		pad = new LogitechGamingPad(0);

		//initialize camera objects 
		server = CameraServer.getInstance();
		camera = new UsbCamera("camera", 0);
		camera2 = new UsbCamera("camera2", 1);
		visionCamera = new AxisCamera("visionCamera", "10.40.79.88");
		
		//set camera values
		camera.setResolution(160, 120);
		camera2.setResolution(160, 120);
		visionCamera.setResolution(480, 360);
		server.startAutomaticCapture(camera);
		server.startAutomaticCapture(camera2);
		server.startAutomaticCapture(visionCamera);
		
		//initialize ultrasonic sensor
		sonicSensor = new LVMaxSonarEZUltrasonic(C.Port.SONIC_SENSOR_INPUT_PORT);

		//initalize timer
		timer = new Timer();

		//initialize Network Tables and get arrays from Contours Report
		table = NetworkTable.getTable("GRIP/myContoursReport");
		centerY = table.getNumberArray("centerY", defaultValue);
		centerX = table.getNumberArray("centerX", defaultValue);
		
		//initialize PDP
		pdp = new PowerDistributionPanel();
		pdp.clearStickyFaults();

		//set joystick toggles
		reverseDirectionToggle = false;
		conveyorToggle = false;
	}
	
	public void autonomous() {
		timer.start();
		int autonomousCase = C.Auto.SELF_CORRECT_DRIVE;

		while (isAutonomous() && isEnabled()) {
			centerY = table.getNumberArray("centerY", defaultValue);
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

				if (
					 * talonDrive.getAngle() < 65 && talonDrive.getAngle() > 55
					 * &&
					  centerY.length == 2 && centerX.length == 2) {
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
		}
	}

	public void operatorControl() {
		boolean stop = false;
		boolean reverseOrNah = false; 
		//boolean conveyorOnOrNah = false;
		talonDrive.resetGyro();
		resetEncoders();
		
		while (isOperatorControl() && isEnabled()) {
			SmartDashboard.putDouble("Gyroscope value: ", talonDrive.getAngle());

			slowTrigger = pad.getLeftTriggerValue();
			//winch = pad.getRightTriggerValue();
			eStop1Pressed = pad.getBackButton();
			reverseDirectionPressed = pad.getAButton();
			conveyorPressed = pad.getLeftBumper();
			//sprintLeft = pad.getRawButton(9);
			//sprintRight = pad.getRawButton(10);
			
			if (eStop1Pressed)
				stop = true;
			
			/*
			if (reverseDirectionPressed && !reverseDirectionToggle)
			{
				reverseDirectionToggle = true;
				reverseOrNah = !reverseOrNah; 	
			}
			if (!reverseDirectionPressed)
				reverseDirectionToggle = false; 
			
			if (sprintLeft && sprintRight && reverseOrNah && !stop)
				talonDrive.tankDrive(pad.getLeftAnalogY(),
						pad.getRightAnalogY());
			
			if (sprintLeft && sprintRight && !reverseOrNah && !stop)
				talonDrive.reverseTankDrive(pad.getLeftAnalogY(),
						pad.getRightAnalogY());
			*/
			
			if (/*reverseOrNah && */ !stop)
				talonDrive.tankDrive(pad.getRightAnalogY(),
						pad.getLeftAnalogY());
			
			/*else if (!reverseOrNah && !stop)
				talonDrive.reverseTankDrive(pad.getLeftAnalogY(),
						pad.getRightAnalogY());
						
			if (conveyorPressed && !conveyorToggle)
			{
				conveyorToggle = true;
				reverseOrNah = !reverseOrNah; 	
			}
			if (!conveyorPressed)
				conveyorToggle = false; 

			if (conveyorOnOrNah)
				conveyorMotor.set(speed);
			else if (!conveyorOnOrNah)
				conveyorMotor.set(C.Speed.STOP_SPEED);
			
			winchMotor.set(winch);
			*/
			SmartDashboard.putDouble("Left Axis: ", pad.getLeftAnalogY());
			SmartDashboard.putDouble("Right Axis: ", pad.getRightAnalogY());
			SmartDashboard.putDouble("Encoder Left Back: ", talonDrive.getBackLeftDrive().getEncPosition());
			SmartDashboard.putDouble("Encoder Right Back: ", talonDrive.getBackRightDrive().getEncPosition());
			SmartDashboard.putDouble("Encoder Left Front: ", talonDrive.getFrontLeftDrive().getEncPosition());
			SmartDashboard.putDouble("Encoder Right Front: ", talonDrive.getFrontRightDrive().getEncPosition());
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

	public void test() {
		while (isTest() && isEnabled()) {
		}
	}
}
