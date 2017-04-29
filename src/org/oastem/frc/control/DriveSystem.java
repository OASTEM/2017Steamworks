package org.oastem.frc.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

import org.oastem.frc.sensor.*;


/**
 * Class to control the entirety of the drive train of an FRC robot.
 * Originally designed by Kevin Tran for FRC 4079 in 2013: Ultimate Ascent.
 * Modified by future years of FRC 4079.
 * 
 * @author KTOmega
 */
public class DriveSystem {
    // Constants.
    protected static final int NUM_ITEMS = 12;
    protected static final double DISTANCE_PER_REVOLUTION = 6 * Math.PI; // FOR DEFAULT DRIVE WHEELS 
    protected static final double AUTO_DRIVE_POWER = 0.75; // percentage between 0 and 1
    protected static final double CORRECTION = 0.50;
    protected static final double BUFFER = 0.1;
    protected static final double BUFFER_ANGLE = 5;
    protected static final double COMPENSATION = 7;
    
    // Singleton design pattern: instance of this class.
    // Only one drive system is allowed per robot - 
    // if any class needs it, it can call the getInstance()
    // method to use it.
    private static DriveSystem instance;
    
    private RobotDrive drive;
    @Deprecated private Victor[] raw;
    private boolean hasSecondary = false;
    private RobotDrive drive2;
    
    protected QuadratureEncoder encRight;
    protected QuadratureEncoder encLeft;
    protected FRCGyroAccelerometer gyroAccel;
    
    protected DriveSystem() {
        //raw = new Victor[NUM_ITEMS];  DEPRECATED
        gyroAccel = new FRCGyroAccelerometer();
    }
    
    public static DriveSystem getInstance() {
        if (instance == null) {
            instance = new DriveSystem();
        }
        
        return instance;
    }
    
    /**** ENCODER STUFF ****/
    public void initializeEncoders(int rightChannelA, int rightChannelB, boolean rightReflected,
    								int leftChannelA, int leftChannelB, boolean leftReflected, double pulsesPerRev) {
        encRight = new QuadratureEncoder(rightChannelA, rightChannelB, rightReflected, 4, pulsesPerRev);
        encLeft = new QuadratureEncoder(leftChannelA, leftChannelB, leftReflected, 4, pulsesPerRev);
        encRight.setDistancePerPulse(DISTANCE_PER_REVOLUTION);
        encLeft.setDistancePerPulse(DISTANCE_PER_REVOLUTION);
        encRight.reset();
        encLeft.reset();
    }
    
    public double getRightEnc()
    {
    	return encRight.getDistance();
    }
    
    public double getLeftEnc()
    {
    	return encLeft.getDistance();
    }
    
    public double getRateRightEnc()
    {
    	return encRight.getRate();
    }
    
    public double getRateLeftEnc()
    {
    	return encLeft.getRate();
    }
    
    /**** GYRO/ACCELEROMETER STUFF ****/
    public double getAngle()
    {
    	return gyroAccel.getGyroAngle();
    }
    
    public void resetGyro()
    {
    	gyroAccel.resetGyro();
    }
    
    // ADD ACCELEROMETER GETS
    
    
    /**** DRIVE STUFF ****/
    public void initializeDrive(int leftFront, int leftRear, int rightFront, int rightRear) {
        drive = new RobotDrive(leftFront, leftRear, rightFront, rightRear);
    }
    
    public void initializeDrive(int left, int right){
        drive = new RobotDrive(left, right);
    }
    
    public void setDrive(RobotDrive rd) {
        drive = rd;
    }
    
    public void initializeSecondaryDrive(int l2, int r2) {
        drive2 = new RobotDrive(l2, r2);
        hasSecondary = true;
    }
    
    public void setSecondaryDriver(RobotDrive rd) {
        drive2 = rd;
        hasSecondary = true;
    }
    
    public void arcadeDrive(Joystick joystick){
        drive.arcadeDrive(joystick);
    }
    
    public void arcadeDrive(double forward, double turn) {
        drive.arcadeDrive(forward, turn);
        if (hasSecondary) drive2.arcadeDrive(forward, turn);
    }
    
    public void tankDrive(double x, double y) {
        drive.tankDrive(x, y);
        if (hasSecondary) drive2.tankDrive(x, y);
    }
    
    public void mecanumDrive(double x, double y, double turn, double gyro) {
        drive.mecanumDrive_Cartesian(x, y, turn, gyro);
        if (hasSecondary) drive2.mecanumDrive_Cartesian(x, y, turn, gyro);
    }
    
    /**
     * @deprecated Due to the addition of CAN communication and advancements in actuators,
     *  using just Victors to control the robot is not a viable control method.
     *  DriveSystem is now used purely for drive and each actuator (i.e. an arm or a winch)
     *  should be created as its own object in Robot.java. 
     */
    @Deprecated public void addVictor(int port) {
        raw[port] = new Victor(port);
    }
    
    /**
     * @deprecated Due to the addition of CAN communication and advancements in actuators,
     *  using just Victors to control the robot is not a viable control method.
     *  DriveSystem is now used purely for drive and each actuator (i.e. an arm or a winch)
     *  should be created as its own object in Robot.java. 
     */
    @Deprecated public void set(int vic, double power) {
        raw[vic].set(power);
    }
    
    /**
     * @deprecated Due to the addition of CAN communication and advancements in actuators,
     *  using just Victors to control the robot is not a viable control method.
     *  DriveSystem is now used purely for drive and each actuator (i.e. an arm or a winch)
     *  should be created as its own object in Robot.java. 
     */
    @Deprecated  public double getPwm(int vic) {
        return raw[vic].get();
    }
    
    /**
     * @deprecated Due to the addition of CAN communication and advancements in actuators,
     *  using just Victors to control the robot is not a viable control method.
     *  DriveSystem is now used purely for drive and each actuator (i.e. an arm or a winch)
     *  should be created as its own object in Robot.java. 
     */
    @Deprecated public Victor getVictor(int vic) {
        return raw[vic];
    }
    
    public void setSafety(boolean b) {
        drive.setSafetyEnabled(false);
        if (hasSecondary) drive2.setSafetyEnabled(false);
    }

    public void resetEncoders()
    {
    	encRight.reset();
    	encLeft.reset();
    }
    
    public boolean reverse(double distance) {
        if ( (encRight.getDistance() > (-distance + COMPENSATION) ) || (encLeft.getDistance() > (-distance + COMPENSATION) ) ) {
            //drive.tankDrive(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER);
            /**********/
            keepStraightBackward();
            if (hasSecondary) drive2.tankDrive(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER);
            return false;
        } else {
            drive.tankDrive(0, 0);
            return true;
        }
    }

    public boolean forward(double distance, double startingGyroAngle) {
        // Both sides have not reached distance yet
    	if ( (encRight.getDistance() < (distance - COMPENSATION) ) && (encLeft.getDistance() < (distance - COMPENSATION) ) ) {
            keepStraightForward(startingGyroAngle);
        }
    	// Right side has not reached distance yet
        else if (encRight.getDistance() < (distance - COMPENSATION))
        {
        	drive.tankDrive(0, AUTO_DRIVE_POWER);
        }
    	// Left side has not reached distance yet
        else if (encLeft.getDistance() < (distance - COMPENSATION))
        {
        	drive.tankDrive(AUTO_DRIVE_POWER, 0);
        }
    	// Distance is reached
        else
        {
            drive.tankDrive(0, 0);
            return true;
        }
        return false;
    }
    
    public void keepStraightForward(double straightGyroAngle)
    {
    	double currAngle = gyroAccel.getGyroAngle();
    	if (straightGyroAngle + BUFFER_ANGLE < currAngle)
    	{
    		drive.tankDrive(AUTO_DRIVE_POWER - CORRECTION, AUTO_DRIVE_POWER);
    	}
    	else if (currAngle < straightGyroAngle - BUFFER_ANGLE)
    	{
    		drive.tankDrive(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER - CORRECTION);
    	}
    	else
    	{
    		drive.tankDrive(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER);
    	}
    }
    
    private void keepStraightBackward()
    {
    	double rightVal = encRight.getDistance();
    	double leftVal = encLeft.getDistance();
    	//BACKWARD
    	if (leftVal < rightVal - BUFFER)
    	{
    		drive.tankDrive(AUTO_DRIVE_POWER - CORRECTION, AUTO_DRIVE_POWER);
    	}
    	else if (rightVal < leftVal - BUFFER)
    	{
    		drive.tankDrive(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER - CORRECTION);
    	}
    	else
    	{
    		drive.tankDrive(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER);
    	}
    }
    
    public void setInvertedDouble() {
        drive.setInvertedMotor(MotorType.kRearLeft, true);
        drive.setInvertedMotor(MotorType.kRearRight, true);
    }
    
    public void setInvertedQuad() {
        drive.setInvertedMotor(MotorType.kFrontLeft, true);
        drive.setInvertedMotor(MotorType.kFrontRight, true);
        drive.setInvertedMotor(MotorType.kRearLeft, true);
        drive.setInvertedMotor(MotorType.kRearRight, true);
    }
}
