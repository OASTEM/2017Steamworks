package org.oastem.frc.strong;

import java.util.Arrays;

import org.oastem.frc.LogitechGamingPad;
import org.oastem.frc.control.TalonDriveSystem;
import org.oastem.frc.sensor.LVMaxSonarEZUltrasonic;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	// Ports
	private final int FRONT_LEFT_CAN_DRIVE = 0;
	private final int FRONT_RIGHT_CAN_DRIVE = 2;
	private final int BACK_LEFT_CAN_DRIVE = 1;
	private final int BACK_RIGHT_CAN_DRIVE = 3;
	private final int SONIC_SENSOR_INPUT_PORT = 0;
	private final int CAMERA_RESOLUTION_WIDTH = 480;
	private final int CAMERA_RESOLUTION_HEIGHT = 360; 

	// Values
	private final int DRIVE_ENC_CODE_PER_REV = 2048;
	private final int DRIVE_WHEEL_DIAM = 8;

	// Objects
	private TalonDriveSystem talonDrive = TalonDriveSystem.getInstance();
	private LogitechGamingPad pad;
	private LogitechGamingPad padSupport;
	private SmartDashboard dash;
	private CameraServer server;
	private UsbCamera camera;
	private UsbCamera camera2;
	private AxisCamera visionCamera; 
	private LVMaxSonarEZUltrasonic sonicSensor;
	private Timer timer; 

	// Joystick commands
	private double slowTrigger;
	private boolean eStop1Pressed;
	private boolean eStop2Pressed;

	//Network Table
	private NetworkTable table;
	
	//Camera Stuff
	private double[] defaultValue = new double[0];
	private double[] centerY; 
	private double[] centerX;
	private double centerXCoor = 0;
	private double cameraWidth = 160;
	private double cameraLength = 120; 
	
	//Autonomous Cases
	private final int DRIVE_STRAIGHT = 1;
	private final int MOMENTUM_STOP= 2;
	private final int TURN = 3; 
	private final int STOP_WHEN_CENTERED = 4; 
	private final int SELF_CORRECT_DRIVE = 5; 
	private final int STOP_WHEN_CLOSE_ENOUGH = 6; 
	
	//Autonomous Values
	private final int HALF_RES_LEFT = CAMERA_RESOLUTION_WIDTH/2 - 20;
	private final int HALF_RES_RIGHT = CAMERA_RESOLUTION_WIDTH/2 + 20;
	private final double SWERVE_DRIVE_SPEED = 0.3;
	private final double STRAIGHT_DRIVE_SPEED = 0.2;
	private final double TURN_SPEED = 0.7;
	private final double STOP_SPEED = 0;
	private final int MIN_DISTANCE = 14;
	private final int MAX_DISTANCE = 16; 
	

	public Robot() {
		talonDrive.initializeTalonDrive(FRONT_LEFT_CAN_DRIVE, BACK_LEFT_CAN_DRIVE, FRONT_RIGHT_CAN_DRIVE,
				BACK_RIGHT_CAN_DRIVE, DRIVE_ENC_CODE_PER_REV, DRIVE_WHEEL_DIAM);
		dash = new SmartDashboard();
		talonDrive.calibrateGyro();

		pad = new LogitechGamingPad(0);
		padSupport = new LogitechGamingPad(1);

		//camera objects 
		server = CameraServer.getInstance();
		camera = new UsbCamera("camera1", 1);
		camera2 = new UsbCamera("camera2", 0);
		visionCamera = new AxisCamera("visionCamera", "10.40.79.88");
		camera.setResolution(160, 120);
		camera2.setResolution(160, 120);
		visionCamera.setResolution(480, 360);
		server.startAutomaticCapture(camera);
		server.startAutomaticCapture(camera2);
		server.startAutomaticCapture(visionCamera);
		
		sonicSensor = new LVMaxSonarEZUltrasonic(SONIC_SENSOR_INPUT_PORT);
		
		timer = new Timer();
		
		table = NetworkTable.getTable("GRIP/myContoursReport");
		centerY = table.getNumberArray("centerY", defaultValue);
		centerX  = table.getNumberArray("centerX", defaultValue);

	}

	public void autonomous() {
		timer.start();
		int autonomousCase = DRIVE_STRAIGHT;
		
		while (isAutonomous() && isEnabled()) {
			centerY = table.getNumberArray("centerY", defaultValue);
			centerX = table.getNumberArray("centerX", defaultValue);
			
			switch (autonomousCase)
			{
				case DRIVE_STRAIGHT:
					if (timer.get() < 1)
						talonDrive.reverseTankDrive(STRAIGHT_DRIVE_SPEED, STRAIGHT_DRIVE_SPEED);
					else 
						autonomousCase = MOMENTUM_STOP;
					break;
					
				case MOMENTUM_STOP:
					if (timer.get() < 4)
						talonDrive.tankDrive(STOP_SPEED, STOP_SPEED);
					else 
						autonomousCase = TURN;
					break;
					
				case TURN: 
					talonDrive.tankDrive(TURN_SPEED, -TURN_SPEED);
					if (talonDrive.getAngle() < 65 && talonDrive.getAngle() > 55 && centerY.length == 2 && centerX.length == 2)
						autonomousCase = STOP_WHEN_CENTERED; 
					break;
					
				case STOP_WHEN_CENTERED: 
					centerXCoor = (centerX[0] + centerX[1])/2;
					if (centerXCoor >= HALF_RES_LEFT && centerXCoor <= HALF_RES_RIGHT)
					{
						talonDrive.tankDrive(STOP_SPEED, STOP_SPEED);
						autonomousCase = SELF_CORRECT_DRIVE; 
					}
					break; 
					
				case SELF_CORRECT_DRIVE: 
					if (centerY.length == 2 && centerX.length == 2)
						centerXCoor = (centerX[0] + centerX[1])/2;
					if (!(sonicSensor.getDistance() > MIN_DISTANCE && sonicSensor.getDistance() < MAX_DISTANCE))
					{
						if (centerXCoor >= HALF_RES_LEFT && centerXCoor <= HALF_RES_RIGHT)
							talonDrive.reverseTankDrive(STRAIGHT_DRIVE_SPEED, STRAIGHT_DRIVE_SPEED);
						else if (centerXCoor <= HALF_RES_LEFT)
							talonDrive.reverseTankDrive(SWERVE_DRIVE_SPEED, STOP_SPEED);
						else if (centerXCoor >= HALF_RES_RIGHT)
							talonDrive.reverseTankDrive(STOP_SPEED, SWERVE_DRIVE_SPEED);	
						else if (centerY.length < 2 && centerX.length < 2)
							talonDrive.reverseTankDrive(STRAIGHT_DRIVE_SPEED, STRAIGHT_DRIVE_SPEED);
					}
					else
							autonomousCase = STOP_WHEN_CLOSE_ENOUGH;
					break;
					
				case STOP_WHEN_CLOSE_ENOUGH:
					if (sonicSensor.getDistance() > MIN_DISTANCE && sonicSensor.getDistance() < MAX_DISTANCE)
						talonDrive.tankDrive(STOP_SPEED, STOP_SPEED);
					break;
			}
			SmartDashboard.putNumber("Ultrasonic reading: ", sonicSensor.getDistance());
			SmartDashboard.putNumber("Ultrasonic raw reading; ", sonicSensor.getVoltage());	
		}	
	}

	public void operatorControl() {
		boolean stop = false;
		while (isOperatorControl() && isEnabled()) {
			slowTrigger = pad.getLeftTriggerValue();

			eStop1Pressed = pad.getBackButton() || padSupport.getBackButton();
			eStop2Pressed = pad.getStartButton() || padSupport.getStartButton();

			if (eStop1Pressed && eStop2Pressed)
				stop = true;

			if (!stop) 
				talonDrive.tankDrive(pad.getLeftAnalogY()  * -scaleTrigger(slowTrigger),
						pad.getRightAnalogY() * -scaleTrigger(slowTrigger));
			
			SmartDashboard.putNumber("Ultrasonic reading: ", sonicSensor.getDistance());
			SmartDashboard.putNumber("Ultrasonic raw reading; ", sonicSensor.getVoltage());
			
			SmartDashboard.putNumber("Gyroscope value: ", talonDrive.getAngle());
		}
	}

	private double scaleTrigger(double trigger) {
		return Math.min(1.0, 1.0 - 0.9 * trigger);
	}

	public double predictDistance (double pixels)
	{
		// y = -1.542693 + (2487.625 - -1.542693)/(1 + (x/1.294396)^0.9814597)
		double num = 2487.625 + 1.542693;
		double den = 1 + Math.pow((pixels/1.294396), 0.9814597);
		return (num/den -1.542693);
	}

	public void test() {
	
		
		while (isTest() && isEnabled()){
			dash.putString("mode: ", "test");
			centerY = table.getNumberArray("centerY", defaultValue);
			centerX = table.getNumberArray("centerX", defaultValue);
			dash.putString("centerY: ", Arrays.toString(centerY));
			dash.putString("centerX: ", Arrays.toString(centerX));
			
			if (centerY.length == 2 && centerX.length == 2)
			{
				centerXCoor = (centerX[0] + centerX[1])/2;
				//dash.putNumber("pegX: ", (centerY[0] + centerY[1])/2);
				//dash.putNumber("pegY: ", centerXCoor);
				if (centerXCoor <= 60)
					talonDrive.tankDrive(0.15, 0.15);
				else if (centerXCoor > 60 && centerXCoor <= 70)
					talonDrive.tankDrive(0.12, 0.12);
				else if (centerXCoor >= 90 && centerXCoor < 100)
					talonDrive.tankDrive(-0.12, -0.12);
				else if (centerXCoor >= 100)
					talonDrive.tankDrive(-0.15, -0.15);
				else
					{
						talonDrive.tankDrive(0,0);
						dash.putNumber("Distance between tapes: ", centerX[1]-centerX[0]);
						dash.putNumber("Predicted distance from robot: ", predictDistance(centerX[1]-centerX[0]));
					}
			}
			
			else if (centerY.length == 1 && centerX.length == 1)
			{
				if (centerX[0] <= 80)
					talonDrive.tankDrive(0.15, 0.15);
				else if (centerX[0] >= 80)
					talonDrive.tankDrive(-0.15, -0.15);
			}		
		}
	}
}
