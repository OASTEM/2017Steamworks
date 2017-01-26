package org.oastem.frc.control;

import org.oastem.frc.sensor.FRCGyroAccelerometer;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonDriveSystem {// (:
	// TALON_SRX's
	private CANTalon frontRightDrive;
	private CANTalon frontLeftDrive;
	private CANTalon backRightDrive;
	private CANTalon backLeftDrive;
	private Accelerator accLeft;
	private Accelerator accRight;
	private FRCGyroAccelerometer gyro;
	private int encoderCodePerRev;
	private double wheelCircum;
	private int tick;
	private double startLeft;
	private double startRight;
	private double startAngle;

	private final double DRIVE_POWER = 30;
	private final double COMPENSATION = .5;
	private final double BUFFER_ANGLE = 20;
	// Singleton design pattern: instance of this class.
	// Only one talon drive system is allowed per robot -
	// if any class needs it, it can call the getInstance(:-)
	// method to use it.
	private static TalonDriveSystem instance;

	public static TalonDriveSystem getInstance() {
		if (instance == null) {

			instance = new TalonDriveSystem();
		}

		return instance;
	}

	public TalonDriveSystem() {
		tick = 0;
		startLeft = 0;
		startRight = 0;
		startAngle = 0;
	}

	public void initializeTalonDrive(int leftFront, int leftRear, int rightFront, int rightRear, int pulsesPerRev,
			int wheelDiameter) {
		frontRightDrive = new CANTalon(rightFront);
		frontLeftDrive = new CANTalon(leftFront);
		backRightDrive = new CANTalon(rightRear);
		backLeftDrive = new CANTalon(leftRear);
		
		frontRightDrive.enableBrakeMode(true);
		frontLeftDrive.enableBrakeMode(true);
		backRightDrive.enableBrakeMode(true);
		backLeftDrive.enableBrakeMode(true);
		
		encoderCodePerRev = pulsesPerRev;
		wheelCircum = wheelDiameter * Math.PI;
		accLeft = new Accelerator();
		accRight = new Accelerator();
		gyro = new FRCGyroAccelerometer();
		
		initCan();
	}

	// :-)
	public void initializeTalonDrive(int left, int right, int pulsesPerRev, int wheelDiameter) {
		frontRightDrive = null;
		frontLeftDrive = null;
		backRightDrive = new CANTalon(right);
		backLeftDrive = new CANTalon(left);
		
		backRightDrive.enableBrakeMode(true);
		backLeftDrive.enableBrakeMode(true);
		
		encoderCodePerRev = pulsesPerRev;
		wheelCircum = wheelDiameter * Math.PI;
		accLeft = new Accelerator();
		accRight = new Accelerator();
		gyro = new FRCGyroAccelerometer();
		
		initCan();
		SmartDashboard.putString("Swag", "Dank Dreams");
	}

	private void initCan() {
		if (frontRightDrive != null)
			frontRightDrive.changeControlMode(TalonControlMode.Follower);
		if (frontLeftDrive != null)
			frontLeftDrive.changeControlMode(TalonControlMode.Follower);
		TalonControlMode mode = TalonControlMode.Speed;
		FeedbackDevice encoder = FeedbackDevice.QuadEncoder;
		
		backRightDrive.changeControlMode(mode);
		backRightDrive.setFeedbackDevice(encoder);
		backRightDrive.configEncoderCodesPerRev(encoderCodePerRev);
		backRightDrive.enable();
		backRightDrive.setPID(.6, 0, 0);
		backRightDrive.setF(.534);
		backRightDrive.reverseOutput(true);
		backRightDrive.reverseSensor(true);
		
		backLeftDrive.changeControlMode(mode);
		backLeftDrive.setFeedbackDevice(encoder);
		backLeftDrive.configEncoderCodesPerRev(encoderCodePerRev);
		backLeftDrive.enable();
		backLeftDrive.setPID(.6, 0, 0);
		backLeftDrive.setF(.534);

	}

	/**** GYRO STUFF ****/
	public double getAngle() {
		return gyro.getGyroAngle();
	}

	public void calibrateGyro()
	{
		gyro.calibrateGyro();
	}
	
	public void resetGyro() {
		gyro.resetGyro();
	}
	
	public double getAccelX(){
		return gyro.getAccelX();
	}
	
	public double getAccelY(){
		return gyro.getAccelY();
	}
	
	public double getAccelZ(){
		return gyro.getAccelZ();
	}

	private void changeTalonToSpeed() {
		TalonControlMode mode = TalonControlMode.Speed;
		backLeftDrive.changeControlMode(mode);
		backRightDrive.changeControlMode(mode);
	}

	private void changeTalonToPercent() {
		TalonControlMode mode = TalonControlMode.PercentVbus;
		backLeftDrive.changeControlMode(mode);
		backRightDrive.changeControlMode(mode);
	}

	public void driveStraight(double speed) {
		double currAngle = gyro.getGyroAngle();
		
		if (tick++ == 0)
			startAngle = currAngle;
		
		
		SmartDashboard.putNumber("Start gyro", startAngle);
		
		double diff = Math.abs(currAngle - startAngle);
		double comp = diff * 20 / 100;
		
		SmartDashboard.putNumber("Diff", diff);
		
		if (comp > 1)
			comp = 1;
		
		SmartDashboard.putNumber("Compensation", comp);

		if (currAngle > startAngle) {
			speedTankDrive(speed * (1 - comp), speed, false);
		} else if (currAngle < startAngle) {
			speedTankDrive(speed, speed * (1 - comp), false);
		} else {
			speedTankDrive(speed, speed, false);
		}

	}

	public void speedTankDrive(double leftValuePerMin, double rightValuePerMin, boolean isInInches) {
		changeTalonToSpeed();
		
		double leftRPM = leftValuePerMin;
		double rightRPM = rightValuePerMin;
		
		if (isInInches) {
			leftRPM /= wheelCircum;
			rightRPM /= wheelCircum;
		}
		
		backLeftDrive.set(leftRPM);
		backRightDrive.set(rightRPM);
		
		SmartDashboard.putNumber("Back Left Speed", backLeftDrive.get());
		SmartDashboard.putNumber("Back Right Speed", backRightDrive.get());
		
		slave();
	}// c:

	public void accelTankDrive(double left, double right) {
		changeTalonToPercent();
		
		backLeftDrive.set(accLeft.decelerateValue(accLeft.getSpeed(), left));
		backRightDrive.set(-accRight.decelerateValue(accRight.getSpeed(), right));
		
		SmartDashboard.putNumber("Acc Left Speed", accLeft.getSpeed());
		SmartDashboard.putNumber("Acc Right Speed", accRight.getSpeed());
		
		slave();
	}

	public void tankDrive(double left, double right) {
		changeTalonToPercent();
		
		backLeftDrive.set(left);
		backRightDrive.set(-right); //-right because of motor orientation

		//SmartDashboard.putNumber("Back Left Position", backLeftDrive.getPosition());
		//SmartDashboard.putNumber("Back Right Position", backRightDrive.getPosition());

		//SmartDashboard.putNumber("Back Left Speed", backLeftDrive.getSpeed());
		//SmartDashboard.putNumber("Back Right Speed", backRightDrive.getSpeed());

		
		slave();
	}
	
	public void reverseTankDrive(double left, double right) {
		changeTalonToPercent();
		
		backLeftDrive.set(-left);
		backRightDrive.set(right);
	}
	

	public boolean driveDistance(double distanceInInches, boolean isFoward) {
		changeTalonToSpeed();
		
		double leftDistance = backLeftDrive.getEncPosition() * wheelCircum;
		double rightDistance = backRightDrive.getEncPosition() * wheelCircum;
		double currAngle = gyro.getGyroAngle();
		
		SmartDashboard.putNumber("Gyro", currAngle);
		
		if (tick++ == 0) {
			startLeft = leftDistance;
			startRight = rightDistance;
			startAngle = currAngle;
			
			SmartDashboard.putString("Start values",
					"Left: " + startLeft + "\tRight: " + startRight + "\tAngle: " + startAngle);
		}

		if (isFoward) {
			if (leftDistance < startLeft + distanceInInches) {
				if (currAngle > startAngle + BUFFER_ANGLE)
					backLeftDrive.set(DRIVE_POWER - COMPENSATION);
				else
					backLeftDrive.set(DRIVE_POWER);
			} else {
				backLeftDrive.set(0);
			}

			if (rightDistance < startRight + distanceInInches) {
				if (currAngle < startAngle - BUFFER_ANGLE)
					backRightDrive.set(0);// DRIVE_POWER - COMPENSATION);
				else
					backRightDrive.set(DRIVE_POWER);
			} else {
				backRightDrive.set(0);
			}

			if ((leftDistance >= startLeft + distanceInInches) && (rightDistance >= startRight + distanceInInches)) {
				tick = 0;
				return true;
			}
		} else {
			if (leftDistance > startLeft - distanceInInches) {
				if (currAngle < startAngle - BUFFER_ANGLE)
					backLeftDrive.set(-DRIVE_POWER + COMPENSATION);
				else
					backLeftDrive.set(-DRIVE_POWER);
			} else {
				backLeftDrive.set(0);
			}

			if (rightDistance > startRight - distanceInInches) {
				if (currAngle > startAngle + BUFFER_ANGLE)
					backRightDrive.set(-DRIVE_POWER + COMPENSATION);
				else
					backRightDrive.set(-DRIVE_POWER);
			} else {
				backRightDrive.set(0);
			}

			if ((leftDistance <= startLeft - distanceInInches) && (rightDistance <= startRight - distanceInInches)) {
				tick = 0;
				return true;
			}
		}

		slave();
		return false;

	}

	private void slave() {
		if (frontLeftDrive != null) {
			frontLeftDrive.set(backLeftDrive.getDeviceID());
		}
		if (frontRightDrive != null) {
			frontRightDrive.set(backRightDrive.getDeviceID());
		}
	}

	public void setPID(double p, double i, double d, double f) {
		backLeftDrive.setPID(p, i, d);
		backLeftDrive.setF(f);
		backRightDrive.setPID(p, i, d);
		backRightDrive.setF(f);
	}

	public CANTalon getFrontLeftDrive() {
		return frontLeftDrive;
	}

	public CANTalon getFrontRightDrive() {
		return frontRightDrive;
	}

	public CANTalon getBackLeftDrive() {
		return backLeftDrive;
	}

	// :)
	public CANTalon getBackRightDrive() {
		return backRightDrive;
	}
	
	public void resetTick(){
		tick = 0;
	}
}