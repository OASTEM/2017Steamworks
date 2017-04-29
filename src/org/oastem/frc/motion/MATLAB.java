package org.oastem.frc.motion;


public class MATLAB {
	public static void main (String[] args){
	//ROBOT
	double frameLength = 31.125;
	double frameWidth = 27.5;
	double backToCenter = 15.155;
	double frontToCenter = frameLength - backToCenter;
	double bumper = 3.25;
	
	//FIELD PLACEMENT
	double fieldCenterToFrameEdge = 77;
	//double fieldCenterToFrameEdge = -frameWidth/2;
	
	//FIELD
	double baseToShip = 114.3;
	double shipEdge = 41.0454;
	double shipToPeg = 3.375;
	double peg = 10.5675;
	
	//MIDDLE PEG
	double xMidPegBase = baseToShip - shipToPeg;
	double yMidPegBase = 0;
	double travelToMidPeg = xMidPegBase - frameLength - bumper;
	
	//LEFT PEG
	double xLftPegBase = baseToShip + 0.5*shipEdge*Math.sin(Math.PI/3) - shipToPeg*Math.cos(Math.PI/3);
	double yLftPegBase = 0.5*shipEdge + 0.5*shipEdge*Math.cos(Math.PI/3) + shipToPeg*Math.sin(Math.PI/3);
	
	double xLftPegTip = baseToShip + 0.5*shipEdge*Math.sin(Math.PI/3) - (shipToPeg + peg)*Math.cos(Math.PI/3);
	double yLftPegTip = 0.5*shipEdge + 0.5*shipEdge*Math.cos(Math.PI/3) + (shipToPeg + peg)*Math.sin(Math.PI/3);
	
	//ROBOT CENTER STARTING POSITION
	double xRobotCenterStart = 0.5*frameLength + bumper;
	double yRobotCenterStart = fieldCenterToFrameEdge + 0.5*frameWidth; 
	
	//Robot center end position. We'll assume that the front of the frame will
	// be aligned with the peg base and the heading of the robot will be aligned
	// with the peg.
	double xRobotCenterEnd = xLftPegBase - frontToCenter*Math.cos(Math.PI/3);
	double yRobotCenterEnd = yLftPegBase + frontToCenter*Math.sin(Math.PI/3);
	
	//Calculate trajectory end point relative to (0,0)
	double xTrajectory = xRobotCenterEnd - xRobotCenterStart;
	double yTrajectory = yRobotCenterEnd - yRobotCenterStart;	
	
	System.out.println("travel to midPeg: " + travelToMidPeg + 
				"\nx-trajectory: " + xTrajectory + "\ny-trajectory: " + yTrajectory + "\nx-lftpegbase: " + xLftPegBase
				+ "\ny-lftpegBase: " + yLftPegBase);
	}
}
