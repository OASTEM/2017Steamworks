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
	private CANTalon backRightDrive; //master
	private CANTalon backLeftDrive; //master
	
	private Accelerator accLeft;
	private Accelerator accRight;
	
	private FRCGyroAccelerometer gyro;
	
	private int encoderPulsePerRev;
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
	
	//CALLED
	public void initializeTalonDrive(int leftFront, int leftRear, int rightFront, int rightRear, int pulsesPerRev,
			int wheelDiameter) {
		
		double RATE = 12;
		
		frontRightDrive = new CANTalon(rightFront);
		frontLeftDrive = new CANTalon(leftFront);
		backRightDrive = new CANTalon(rightRear);
		backLeftDrive = new CANTalon(leftRear);
		
		encoderPulsePerRev = pulsesPerRev;
		wheelCircum = wheelDiameter * Math.PI;
		accLeft = new Accelerator();
		accRight = new Accelerator();
		gyro = new FRCGyroAccelerometer();
		
		
		//backRightDrive.setVoltageRampRate(RATE);
		//backLeftDrive.setVoltageRampRate(RATE);
		
		//backRightDrive.setVoltageCompensationRampRate(RATE);
		//backLeftDrive.setVoltageCompensationRampRate(RATE);
		
		backRightDrive.setF(1.900); //1.911
		backLeftDrive.setF(1.88);  //1.866
		backRightDrive.setP(0.000);
		backLeftDrive.setP(0.000);
		backRightDrive.setI(0);
		backLeftDrive.setI(0);
		backRightDrive.setD(0);
		backLeftDrive.setD(0);
		
		initCan();
	}

	// :-)
	public void initializeTalonDrive(int left, int right, int pulsesPerRev, int wheelDiameter) {
		frontRightDrive = null;
		frontLeftDrive = null;
		backRightDrive = new CANTalon(right);
		backLeftDrive = new CANTalon(left);
		
		backRightDrive.enableBrakeMode(true);
		frontLeftDrive.enableBrakeMode(true);
		
		encoderPulsePerRev = pulsesPerRev;
		wheelCircum = wheelDiameter * Math.PI;
		accLeft = new Accelerator();
		accRight = new Accelerator();
		gyro = new FRCGyroAccelerometer();
		
		initCan();
	}

	private void initCan() {
		if (frontRightDrive != null)
			frontRightDrive.changeControlMode(TalonControlMode.Follower);
		if (frontLeftDrive != null) 
			frontLeftDrive.changeControlMode(TalonControlMode.Follower);
		
		TalonControlMode mode = TalonControlMode.PercentVbus;
		FeedbackDevice encoder = FeedbackDevice.QuadEncoder;
		
		backRightDrive.changeControlMode(mode);
		backRightDrive.setFeedbackDevice(encoder);
		backRightDrive.configEncoderCodesPerRev(encoderPulsePerRev);
		backRightDrive.enable();
		backRightDrive.setInverted(true);
		backRightDrive.reverseOutput(true);
		
		backLeftDrive.changeControlMode(mode);
		backLeftDrive.setFeedbackDevice(encoder);
		backLeftDrive.configEncoderCodesPerRev(encoderPulsePerRev);
		backLeftDrive.enable();
		//backLeftDrive.setInverted(true);
		backLeftDrive.reverseOutput(true);
		
		backLeftDrive.configNominalOutputVoltage(+0.0f, -0.0f);
		backLeftDrive.configPeakOutputVoltage(+12.0f, -12.0f);
		
		backRightDrive.configNominalOutputVoltage(+0.0f, -0.0f);
		backRightDrive.configPeakOutputVoltage(+12.0f, -12.0f);
		
		slave();
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
		
		frontLeftDrive.set(leftRPM);
		backRightDrive.set(rightRPM);
		
		SmartDashboard.putNumber("Front Left Speed", frontLeftDrive.get());
		SmartDashboard.putNumber("Back Right Speed", backRightDrive.get());
		
		slave();
	}

	public void accelTankDrive(double left, double right) {
		changeTalonToPercent();
		
		frontLeftDrive.set(accLeft.decelerateValue(accLeft.getSpeed(), left));
		backRightDrive.set(-accRight.decelerateValue(accRight.getSpeed(), right));
		
		SmartDashboard.putNumber("Acc Left Speed", accLeft.getSpeed());
		SmartDashboard.putNumber("Acc Right Speed", accRight.getSpeed());
		
		slave();
	}
	
	//CALLED
	public void tankDrive(double left, double right) {
		//changeTalonToPercent();
		
		backLeftDrive.set(left);
		backRightDrive.set(right); //-right because of motor orientation
		
		slave();
	}
	
	public void reverseTankDrive(double left, double right) {
		changeTalonToPercent();
	
		backLeftDrive.set(-right);
		backRightDrive.set(-left);
	}
	
	public void driveStraightWithGyro ()
	{
		
	}
	

	public boolean driveDistance(double distanceInInches, boolean isFoward) {
		changeTalonToSpeed();
		
		double leftDistance = frontLeftDrive.getEncPosition() * wheelCircum;
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
					frontLeftDrive.set(DRIVE_POWER - COMPENSATION);
				else
					frontLeftDrive.set(DRIVE_POWER);
			} else {
				frontLeftDrive.set(0);
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
					frontLeftDrive.set(-DRIVE_POWER + COMPENSATION);
				else
					frontLeftDrive.set(-DRIVE_POWER);
			} else {
				frontLeftDrive.set(0);
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