package org.oastem.frc.sensor;

import edu.wpi.first.wpilibj.AnalogInput;


public class LVMaxSonarEZUltrasonic {
	AnalogInput ultra;
	
	private final double MILLIVOLTS_TO_INCH_CONVERSION = 5000/512; //(Vcc/inch)
	
	public LVMaxSonarEZUltrasonic (int port){
        ultra = new AnalogInput(port);
        
    }
	
	public double getDistance(){
		return ultra.getVoltage()*1000/MILLIVOLTS_TO_INCH_CONVERSION;
	}
	
	public double getVoltage(){
		return ultra.getVoltage()*1000;
	}
	
}
