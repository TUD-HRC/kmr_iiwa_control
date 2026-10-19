// Copyright 2019 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package kmr_iiwa_control.nodes;

// RoboticsAPI
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.executionModel.IExecutionContainer;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainerListener;
import com.kuka.roboticsAPI.motionModel.SplineJP;
import com.kuka.roboticsAPI.motionModel.SplineMotionJP;

import kmr_iiwa_control.motion.PTPpoint;
import kmr_iiwa_control.nodes.KMP_commander.MonitorEmergencyStopThread;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

// Java util
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class LBR_commander extends Node{
	
	// Robot Specific
	LBR lbr;
	
	// Implemented node classes
	LBR_sensor_reader lbr_sensor_reader;
	LBR_status_reader lbr_status_reader;
	KMP_status_reader kmp_status_reader;
	
	// Motion: LBR
	private JointPosition jointPos;
	IMotionContainer currentmotion;
	private static final double DEFAULT_VELOCITY = 0.2;
	AbstractFrame drivePos;
	ICommandContainer LBR_currentMotion;
	private int jointCount;
	private static final double zero = Math.pow(10,-40);

	
	// Spline Motion
	double[] poses;
	double[] velocities;
	double[] accelerations;
	private List<SplineMotionJP<?>> splineSegments = new ArrayList<SplineMotionJP<?>>();

	// Thread handling
	public volatile boolean waiting = false;

	public LBR_commander(int port,LBR robot, String ConnectionType, AbstractFrame drivepos) {
		super(port, ConnectionType, "LBR commander");
		
		this.lbr = robot;
		this.drivePos = drivepos;
		
		jointCount=lbr.getJointCount();
		poses = new double[jointCount];
		velocities = new double[jointCount];
		accelerations = new double[jointCount];
		
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect LBR command node...");
			Thread monitorLBRCommandConnections = new MonitorLBRCommandConnectionsThread();
			monitorLBRCommandConnections.start();
			}else {
				setisLBRConnected(true);
			}

	}
	
	@Override
	public void run() {
		Thread emergencyStopThread = new MonitorEmergencyStopThread();
		emergencyStopThread.start();

		jointPos = lbr.getCurrentJointPosition();
		
		while(isNodeRunning())
		{   
			//while(waiting){}

			String command = socket.receive_message(); 
	    	String[] commandSplitted = command.split(" ");
			//System.out.println("received " + command );

			if (getShutdown()) {
				System.out.println("Can not command: Shutdown engaged");
				break;
			} if (closed) {
				System.out.println("Can not command: Is closed");
				break;

			} if(! isNodeRunning()) {
				System.out.println("Can not command: Node is not running");
				break;
			} 
				
			if (getEmergencyStop()){
				System.out.println("Can not command: Emergency stop engaged");
				break;
			}

			//coomands
			if ((commandSplitted[0]).equals("Shutdown")){

				System.out.println("LBR RECEIVED SHUTDOWN");
				setShutdown(true);

			} else if ((commandSplitted[0]).equals("CancelMotion")) {
				if(getisLBRMoving()){
					currentmotion.cancel();
					setisLBRMoving(false);
					jointPos.set(lbr.getCurrentJointPosition());
				} 

			} else if ((commandSplitted[0]).equals("SetJointPosition")) {
				
				application.KMRiiwaControlApp.StateChange = true;
				jointPositionMotion(commandSplitted);

			} else if ((commandSplitted[0]).equals("SetSingleJointPosition")){
				singleJointPositionMotion(commandSplitted);	

			} else if ((commandSplitted[0]).equals("SetJointTrajectory")){
				addPointToSegment(command);
			
			} else {
				System.out.println("Command not recognized: " + commandSplitted[0] );
			}

	    }
    }

	private void addPointToSegment(String commandstr){
		String []lineSplt = commandstr.split(">");
		String pointType = lineSplt[1];
		
		// Skip start point as velocity and acceleration is 0
		if(pointType.equals("StartPoint")){
			
			splineSegments.clear();
			System.out.println("Startpoint received");
			
			setisPathFinished(false);
			
			// Declares state change.
			application.KMRiiwaControlApp.StateChange = true;
		} else {
			// Read message
			for(int i = 0; i < jointCount ; ++i){
				poses[i] = Double.parseDouble(lineSplt[2].split(" ")[i]);
				if(pointType.equals("WayPoint")){
					velocities[i]= saturate(Math.abs(Double.parseDouble(lineSplt[3].split(" ")[i])));
					double accel = saturate(Math.abs(Double.parseDouble(lineSplt[4].split(" ")[i])));
					if(accel==0.0){accel = zero;}
					accelerations[i] = accel;
				}
			}
			PTPpoint ptp = new PTPpoint(pointType, new JointPosition(poses), velocities, accelerations);
			splineSegments.add(ptp.getPTP());
			
			if(pointType.equals("EndPoint")){
				System.out.println("endpoint received");
				if( !(getEmergencyStop()) && !splineSegments.isEmpty() && isNodeRunning()) {
					SplineJP spline = new SplineJP(splineSegments.toArray(new SplineMotionJP<?>[splineSegments.size()]));
					currentmotion = lbr.moveAsync(spline, new SplineMotionListener());
				}
			}
		}
	}

	private double saturate(double value){
		if (value > 1) { 
			value=1.0; 
		}
		return value;
	}

	// commandSplitted[0] = 'SetJointPosition'
	// commandSplitted[1:N] = 'jp_1, ... , jp_7'
	private void jointPositionMotion(String[] commandSplitted) {

		//validate
		if (commandSplitted.length != lbr.getJointCount() + 1) {
			System.err.println("ERROR: 'SetJointPosition' command format error, expected lenght " + (lbr.getJointCount() + 1)  
				+ " received: " + commandSplitted.length);
			return;
		}

		//j_limits = lbr.getJointLimits();

		String[] jointPositionCmdStr = Arrays.copyOfRange(commandSplitted, 1, commandSplitted.length);
		double[] jointPositionCmdDouble = new double[jointPositionCmdStr.length]; 

		for (int i = 0; i < jointPositionCmdDouble.length; i++) {
			try {
				jointPositionCmdDouble[i] = Double.parseDouble(jointPositionCmdStr[i]);
			} catch (NumberFormatException e) {
				System.err.println("Error: '" + jointPositionCmdStr[i] + "' is not a valid number.");
				return; 
			}
		}

		JointPosition jointPosCmd = new JointPosition(jointPositionCmdDouble); // one value per LBR joint, in radians
		if(getisLBRMoving()){ //is this necessary?
			this.currentmotion.cancel();
		}
		this.currentmotion = lbr.moveAsync(
			ptp(jointPosCmd).setJointVelocityRel(DEFAULT_VELOCITY)
		);
		setisLBRMoving(true);

	}

	//commandSplitted[0] = 'SetSingleJointPosition'
	//commandSplitted[1] = jointIndex (starting from 1)
	//commandSplitted[2] = positionCommand
	private void singleJointPositionMotion(String[] commandSplitted) {

		//validate
		if (commandSplitted.length != 3) {
			System.err.println("ERROR: 'SetSingleJointPosition' command format error, expected lenght " + 3 
				+ " received: " + commandSplitted.length);
			return;
		}

		int jointIndex;
		double jointAngle;
		try {
			jointIndex = Character.getNumericValue(commandSplitted[1].charAt(0)) - 1 ;
		} catch (Exception e) {
			System.err.println("Invalid joint index: " + Arrays.toString(commandSplitted));
			return;
		}
		try {
			jointAngle = Double.parseDouble(commandSplitted[2]);
		} catch (Exception e) {
			System.err.println("Invalid joint angle: " + Arrays.toString(commandSplitted));
			return;
		}

		JointPosition jointPosCmd = new JointPosition(this.jointPos);

		if(!(jointPosCmd.get(jointIndex) == jointAngle)) {
			jointPosCmd.set(jointIndex, jointAngle);
			if(getisLBRMoving()){
				if(!(currentmotion==null)){
					currentmotion.cancel();
				}					
			}
			//System.out.println("Moving to " + jointPosCmd.toString());
			currentmotion = lbr.moveAsync(ptp(jointPosCmd).setJointVelocityRel(DEFAULT_VELOCITY));
			setisLBRMoving(true);

		} else {
			System.out.println("Joint " + jointIndex + " already in the commanded position (current: " 
				+ jointPosCmd.get(jointIndex) + " commanded: " + jointAngle);
		}	

	}
	
	public class MonitorEmergencyStopThread extends Thread {
		public void run(){
			while(isNodeRunning()) {
				if (getEmergencyStop()){
					if (getisLBRMoving()){
						System.out.println("Lbr emergencythread");
						if(!(currentmotion==null)){
							currentmotion.cancel();
						}
						jointPos.set(lbr.getCurrentJointPosition());
					}
					setisLBRMoving(false);
				}
			}
		}
	}
	
	public class MonitorLBRCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if(getisKMPConnected()) {
					timeout = connection_timeout;
				}
				createSocket();
				if (isSocketConnected()){
					setisLBRConnected(true);
					break;
				}
				try {
					Thread.sleep(timeout);
				} catch (InterruptedException e) {
					System.out.println("Waiting for connection to LBR commander node ..");
				}
			}
			if(!closed){
				System.out.println("Connection with LBR Command Node OK!");
				runmainthread();
			}	
		}
	}
	
	public class SplineMotionListener implements IMotionContainerListener {
		public SplineMotionListener() { 
		  }
		@Override
		public void onStateChanged(IExecutionContainer arg0, ExecutionState arg1) {
		}

		@Override
		public void containerFinished(IMotionContainer arg0) {
			System.out.println("Container finished!");
			setisLBRMoving(false);
			setisPathFinished(true);
			splineSegments.clear();
		}

		@Override
		public void motionFinished(IMotion arg0) {
		}

		@Override
		public void motionStarted(IMotion arg0) {
			setisLBRMoving(true);
		}
	}
	
	@Override
	public void close(){
		closed = true;
		try {
			jointPos.set(lbr.getCurrentJointPosition());
		} catch(Exception e){
			System.out.println("No motion to end!");
		}

		if(getisLBRMoving()){
			try{
				currentmotion.cancel();
			} catch(Exception e){
				System.out.println("LBR could not stop motion: " +e);
			}
		}
		try{
			this.socket.close();
		} catch(Exception e){
			System.out.println("Could not close LBR commander connection: " +e);
		}
		System.out.println("LBR command closed!");
	}
}