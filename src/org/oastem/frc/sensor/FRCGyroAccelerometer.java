package org.oastem.frc.sensor;

import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class FRCGyroAccelerometer {
	private static final double DRIFT_PER_SECOND = .0161803398875;// 0.333/60; 
	private long lastUpdateTime = 0;
	private double [] accelXAverage;
	private double [] accelYAverage;
	private double [] accelZAverage;
	private int indexX;
	private int indexY;
	private int indexZ;
	private double accelX;
	private double accelY;
	private double accelZ;
	private double sumX;
	private double sumY;
	private double sumZ;
	private boolean isOverOneLoopX;
	private boolean isOverOneLoopY;
	private boolean isOverOneLoopZ;

	private ADXRS450_Gyro gyro;
	private ADXL362 accel;

	public FRCGyroAccelerometer() {
		gyro = new ADXRS450_Gyro();
		accel = new ADXL362(Accelerometer.Range.k8G);
		//gyro.calibrate();
		lastUpdateTime = System.currentTimeMillis();
		accelXAverage = new double[100];
		accelYAverage = new double[100];
		accelZAverage = new double[100];
		indexX = 0;
		indexY = 0;
		indexZ = 0;
		accelX = getRawAccelX();
		accelY = getRawAccelY();
		accelZ = getRawAccelZ();
		sumX = 0;
		sumY = 0;
		sumZ = 0;
		isOverOneLoopX = false;
		isOverOneLoopY = false;
		isOverOneLoopZ = false;
	}

	public void resetGyro() {
		gyro.reset();
		lastUpdateTime = System.currentTimeMillis();
	}
	
	public void calibrateGyro()
	{
		gyro.calibrate();
	}

	public double getGyroAngle() {
		long currentTime = System.currentTimeMillis();
		double value = gyro.getAngle()- DRIFT_PER_SECOND * (currentTime - lastUpdateTime) / 1000.0;
		return value;// averageGyroValue(value);
	}
	
	public double getRawGyro()
	{
		return gyro.getAngle();
	}
	
	public double getRawAccelX()
	{
		return accel.getX();
	}
	
	public double getRawAccelY()
	{
		return accel.getY();
	}
	
	public double getRawAccelZ()
	{
		return accel.getZ();
	}
	
	public void freeRawAccel()
	{
		accel.free();
	}
	
	public double getAccelX()
	{
		if (indexX > 99) 
		{
			indexX = 0;
			isOverOneLoopX = true;
		}
		if ( isOverOneLoopX )
			sumX -= accelXAverage[indexX];
		accelXAverage[indexX] = getRawAccelX()-accelX;
		sumX += accelXAverage[indexX];
		indexX++;
		return (isOverOneLoopX) ? sumX/accelXAverage.length : sumX/indexX+1;
	}
	
	public double getAccelY()
	{
		if (indexY > 99 )
		{
			indexY = 0;
			isOverOneLoopY = true;
		} 
		if ( isOverOneLoopY )
			sumY -= accelYAverage[indexY];
		accelYAverage[indexY] = getRawAccelY()-accelY;		
		sumY += accelYAverage[indexY];
		indexY++;
		return (isOverOneLoopY) ? sumY/accelYAverage.length : sumY/indexY+1;
	}
	
	public double getAccelZ()
	{
		if (indexZ > 99 )
		{
			indexZ = 0; 
			isOverOneLoopZ = true;
		}
		if ( isOverOneLoopZ )
			sumZ -= accelZAverage[indexZ];
		accelZAverage[indexZ] = getRawAccelZ()-accelZ;
		sumZ += accelZAverage[indexZ];
		indexZ++;
		return (isOverOneLoopZ) ? sumZ/accelZAverage.length : sumZ/indexZ+1;
	}
	/*private double averageGyroValue (double gyroValue)
	{
		
	}*/

}
