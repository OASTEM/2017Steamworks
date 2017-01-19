package org.oastem.frc.strong;

import java.util.Arrays;

import org.oastem.frc.LogitechGamingPad;
import org.oastem.frc.control.TalonDriveSystem;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled cnby the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions . If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {
	// Ports
	private final int FRONT_LEFT_CAN_DRIVE = 0;
	private final int FRONT_RIGHT_CAN_DRIVE = 2;
	private final int BACK_LEFT_CAN_DRIVE = 1;
	private final int BACK_RIGHT_CAN_DRIVE = 3;

	// Values
	private final int DRIVE_ENC_CODE_PER_REV = 2048;
	private final int DRIVE_WHEEL_DIAM = 8;

	// Objects
	private TalonDriveSystem talonDrive = TalonDriveSystem.getInstance();
	private LogitechGamingPad pad;
	private LogitechGamingPad padSupport;
	private SmartDashboard dash;
	private UsbCamera camera;
	private CameraServer server;

	// Joystick commands
	private double slowTrigger;
	private boolean eStop1Pressed;
	private boolean eStop2Pressed;

	//Network Table
	private NetworkTable table;


	public Robot() {
		talonDrive.initializeTalonDrive(FRONT_LEFT_CAN_DRIVE, BACK_LEFT_CAN_DRIVE, FRONT_RIGHT_CAN_DRIVE,
				BACK_RIGHT_CAN_DRIVE, DRIVE_ENC_CODE_PER_REV, DRIVE_WHEEL_DIAM);
		dash = new SmartDashboard();
		talonDrive.calibrateGyro();

		pad = new LogitechGamingPad(0);
		padSupport = new LogitechGamingPad(1);

		server = CameraServer.getInstance();
		camera = new UsbCamera("camera", 0);
		camera.setResolution(320, 240);
		server.startAutomaticCapture(camera);
		
		table = NetworkTable.getTable("GRIP/myContoursReport");

	}

	/*
	public void robotInit() {
		dash = new SmartDashboard();
		talonDrive.calibrateGyro();

		pad = new LogitechGamingPad(0);
		padSupport = new LogitechGamingPad(1);

		camera = CameraServer.getInstance();
		camera.startAutomaticCapture();
	}*/

	public void autonomous() {
		double[] defaultValue = new double[0];
		double[] centerY = table.getNumberArray("centerY", defaultValue);
		double[] centerX = table.getNumberArray("centerX", defaultValue);
		double centerXCoor = 0;
		double cameraWidth = 160;
		double cameraLength = 120; 
		
		while (isAutonomous() && isEnabled()) {
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

	public void operatorControl() {
		boolean stop = false;
		while (isOperatorControl() && isEnabled()) {
			slowTrigger = pad.getLeftTriggerValue();

			eStop1Pressed = pad.getBackButton() || padSupport.getBackButton();
			eStop2Pressed = pad.getStartButton() || padSupport.getStartButton();

			if (eStop1Pressed && eStop2Pressed)
				stop = true;

			if (!stop) 
				talonDrive.tankDrive(pad.getLeftAnalogY() * -scaleTrigger(slowTrigger),
						pad.getRightAnalogY() * -scaleTrigger(slowTrigger));
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
		}
	}
}
