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
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;

import kmr_iiwa_control.motion.PTPpoint;

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
	private volatile JointPosition jointPos;
	private volatile IMotionContainer currentmotion;
	private volatile ISmartServoRuntime jointServoRuntime;
	private static final double DEFAULT_VELOCITY = 0.2;
	private static final double DEFAULT_ACCELERATION = 0.2;
	private static final double JOINT_SERVO_MINIMUM_TRAJECTORY_TIME = 20e-3;
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
			if (command == null || command.trim().length() == 0) {
				System.err.println("Ignoring empty LBR command");
				continue;
			}
	    	String[] commandSplitted = command.trim().split("\\s+");
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

			// Shutdown must remain available while the emergency-stop flag is active.
			if ((commandSplitted[0]).equals("Shutdown")){

				System.out.println("LBR RECEIVED SHUTDOWN");
				setShutdown(true);
				break;

			}

			if (getEmergencyStop()){
				System.out.println("Ignoring command while emergency stop is engaged");
				continue;
			}

			if ((commandSplitted[0]).equals("CancelMotion")) {
				stopCurrentMotion();
				setLBRMotionActive(false);

			} else if ((commandSplitted[0]).equals("SetJointPosition")) {
				jointPositionMotion(commandSplitted);

			} else if ((commandSplitted[0]).equals("SetJointServoPosition")) {
				streamJointPosition(commandSplitted);

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
		String []lineSplt = commandstr.split(">", -1);
		if (lineSplt.length < 2) {
			rejectTrajectoryPoint("Missing trajectory point type");
			return;
		}
		String pointType = lineSplt[1].trim();
		
		// Skip start point as velocity and acceleration is 0
		if(pointType.equals("StartPoint")){
			stopCurrentMotion();
			
			splineSegments.clear();
			System.out.println("Startpoint received");
			
			setLBRMotionActive(true);
			return;
		}

		int expectedFields;
		if (pointType.equals("WayPoint")) {
			expectedFields = 5;
		} else if (pointType.equals("EndPoint")) {
			expectedFields = 3;
		} else {
			rejectTrajectoryPoint("Unrecognized trajectory point type: " + pointType);
			return;
		}
		if (lineSplt.length != expectedFields) {
			rejectTrajectoryPoint("Invalid " + pointType + " format: expected "
				+ expectedFields + " fields, received " + lineSplt.length);
			return;
		}

		double[] pointPoses = parseJointValues(lineSplt[2], "trajectory position");
		if (pointPoses == null || !areJointPositionsWithinLimits(pointPoses, "trajectory position")) {
			rejectTrajectoryPoint(null);
			return;
		}

		double[] pointVelocities = velocities;
		double[] pointAccelerations = accelerations;
		if (pointType.equals("WayPoint")) {
			pointVelocities = parseJointValues(lineSplt[3], "trajectory velocity");
			pointAccelerations = parseJointValues(lineSplt[4], "trajectory acceleration");
			if (pointVelocities == null || pointAccelerations == null) {
				rejectTrajectoryPoint(null);
				return;
			}
			for (int i = 0; i < jointCount; ++i) {
				pointVelocities[i] = saturate(Math.abs(pointVelocities[i]));
				double accel = saturate(Math.abs(pointAccelerations[i]));
				pointAccelerations[i] = accel == 0.0 ? zero : accel;
			}
		}

		System.arraycopy(pointPoses, 0, poses, 0, jointCount);
		if (pointType.equals("WayPoint")) {
			System.arraycopy(pointVelocities, 0, velocities, 0, jointCount);
			System.arraycopy(pointAccelerations, 0, accelerations, 0, jointCount);
		}
		PTPpoint ptp = new PTPpoint(pointType, new JointPosition(pointPoses), pointVelocities, pointAccelerations);
		splineSegments.add(ptp.getPTP());
		
		if(pointType.equals("EndPoint")){
			System.out.println("endpoint received");
			if( !(getEmergencyStop()) && !splineSegments.isEmpty() && isNodeRunning()) {
				SplineJP spline = new SplineJP(splineSegments.toArray(new SplineMotionJP<?>[splineSegments.size()]));
				currentmotion = lbr.moveAsync(spline, new SplineMotionListener());
			}
		}
	}

	private void rejectTrajectoryPoint(String reason) {
		if (reason != null) {
			System.err.println("Rejecting LBR trajectory point: " + reason);
		}
		splineSegments.clear();
		setLBRMotionActive(false);
	}

	private double[] parseJointValues(String valueList, String valueName) {
		String trimmedValues = valueList.trim();
		if (trimmedValues.length() == 0) {
			System.err.println("Invalid " + valueName + ": no values supplied");
			return null;
		}
		return parseJointValues(trimmedValues.split("\\s+"), 0, valueName);
	}

	private double[] parseJointValues(String[] values, int startIndex, String valueName) {
		int valueCount = values.length - startIndex;
		if (valueCount != jointCount) {
			System.err.println("Invalid " + valueName + ": expected " + jointCount
				+ " values, received " + valueCount);
			return null;
		}

		double[] parsedValues = new double[jointCount];
		for (int i = 0; i < jointCount; ++i) {
			try {
				parsedValues[i] = Double.parseDouble(values[startIndex + i]);
			} catch (NumberFormatException e) {
				System.err.println("Invalid " + valueName + " for joint " + (i + 1)
					+ ": '" + values[startIndex + i] + "' is not a valid number");
				return null;
			}
			if (Double.isNaN(parsedValues[i]) || Double.isInfinite(parsedValues[i])) {
				System.err.println("Invalid " + valueName + " for joint " + (i + 1)
					+ ": value must be finite");
				return null;
			}
		}
		return parsedValues;
	}

	private boolean areJointPositionsWithinLimits(double[] positions, String valueName) {
		JointPosition minPositions = lbr.getJointLimits().getMinJointPosition();
		JointPosition maxPositions = lbr.getJointLimits().getMaxJointPosition();
		for (int i = 0; i < jointCount; ++i) {
			if (positions[i] < minPositions.get(i) || positions[i] > maxPositions.get(i)) {
				System.err.println("Invalid " + valueName + " for joint " + (i + 1)
					+ ": " + positions[i] + " is outside [" + minPositions.get(i)
					+ ", " + maxPositions.get(i) + "] radians");
				return false;
			}
		}
		return true;
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
		if (commandSplitted.length != jointCount + 1) {
			System.err.println("ERROR: 'SetJointPosition' command format error, expected "
				+ (jointCount + 1) + " values including command, received: " + commandSplitted.length);
			return;
		}

		double[] jointPositionCmdDouble = parseJointValues(commandSplitted, 1, "joint position");
		if (jointPositionCmdDouble == null
				|| !areJointPositionsWithinLimits(jointPositionCmdDouble, "joint position")) {
			return;
		}

		JointPosition jointPosCmd = new JointPosition(jointPositionCmdDouble); // one value per LBR joint, in radians
		moveToJointPosition(jointPosCmd);

	}

	private synchronized void moveToJointPosition(JointPosition target) {
		stopCurrentMotion();

		splineSegments.clear();
		setLBRMotionActive(true);
		jointPos = new JointPosition(target);
		currentmotion = lbr.moveAsync(
			ptp(target).setJointVelocityRel(DEFAULT_VELOCITY),
			new JointPositionMotionListener()
		);
		setisLBRMoving(true);
	}

	private void setLBRMotionActive(boolean active) {
		boolean wasActive = !getisPathFinished();
		if (wasActive != active) {
			setisPathFinished(!active);
			application.KMRiiwaControlApp.StateChange = true;
		}
	}

	private synchronized void streamJointPosition(String[] commandSplitted) {
		if (commandSplitted.length != jointCount + 1) {
			System.err.println("ERROR: 'SetJointServoPosition' command format error, expected "
				+ (jointCount + 1) + " values including command, received: " + commandSplitted.length);
			return;
		}

		double[] targetValues = parseJointValues(commandSplitted, 1, "joint servo position");
		if (targetValues == null
				|| !areJointPositionsWithinLimits(targetValues, "joint servo position")) {
			return;
		}

		JointPosition target = new JointPosition(targetValues);
		if (jointServoRuntime == null || currentmotion == null || currentmotion.isFinished()) {
			stopCurrentMotion();
			splineSegments.clear();
			setLBRMotionActive(true);

			SmartServo servoMotion = new SmartServo(lbr.getCurrentJointPosition());
			servoMotion.setJointVelocityRel(DEFAULT_VELOCITY);
			servoMotion.setJointAccelerationRel(DEFAULT_ACCELERATION);
			servoMotion.setMinimumTrajectoryExecutionTime(JOINT_SERVO_MINIMUM_TRAJECTORY_TIME);
			currentmotion = lbr.moveAsync(servoMotion, new JointServoMotionListener());
			jointServoRuntime = servoMotion.getRuntime();
		}

		ISmartServoRuntime runtime = jointServoRuntime;
		runtime.updateWithRealtimeSystem();
		runtime.setDestination(target);
		jointPos = new JointPosition(target);
		setisLBRMoving(true);
	}

	private synchronized void stopCurrentMotion() {
		ISmartServoRuntime servoRuntime = jointServoRuntime;
		IMotionContainer motion = currentmotion;
		jointServoRuntime = null;
		currentmotion = null;

		try {
			if (servoRuntime != null) {
				servoRuntime.stopMotion();
			} else if (motion != null && !motion.isFinished()) {
				motion.cancel();
			}
		} catch (Exception e) {
			System.err.println("Could not stop active LBR motion: " + e);
		} finally {
			setisLBRMoving(false);
			try {
				jointPos = lbr.getCurrentJointPosition();
			} catch (Exception e) {
				System.err.println("Could not refresh LBR joint position after stopping: " + e);
			}
		}
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
			jointIndex = Integer.parseInt(commandSplitted[1]) - 1;
		} catch (NumberFormatException e) {
			System.err.println("Invalid joint index: " + Arrays.toString(commandSplitted));
			return;
		}
		if (jointIndex < 0 || jointIndex >= jointCount) {
			System.err.println("Invalid joint index: " + (jointIndex + 1)
				+ "; expected a value from 1 to " + jointCount);
			return;
		}
		try {
			jointAngle = Double.parseDouble(commandSplitted[2]);
		} catch (NumberFormatException e) {
			System.err.println("Invalid joint angle: " + Arrays.toString(commandSplitted));
			return;
		}
		if (Double.isNaN(jointAngle) || Double.isInfinite(jointAngle)) {
			System.err.println("Invalid joint angle: value must be finite");
			return;
		}
		JointPosition minPositions = lbr.getJointLimits().getMinJointPosition();
		JointPosition maxPositions = lbr.getJointLimits().getMaxJointPosition();
		if (jointAngle < minPositions.get(jointIndex) || jointAngle > maxPositions.get(jointIndex)) {
			System.err.println("Invalid joint angle for joint " + (jointIndex + 1)
				+ ": " + jointAngle + " is outside [" + minPositions.get(jointIndex)
				+ ", " + maxPositions.get(jointIndex) + "] radians");
			return;
		}

		JointPosition jointPosCmd = new JointPosition(this.jointPos);

		if(!(jointPosCmd.get(jointIndex) == jointAngle)) {
			jointPosCmd.set(jointIndex, jointAngle);
			moveToJointPosition(jointPosCmd);

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
						stopCurrentMotion();
						setLBRMotionActive(false);
					}
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
				startNode();
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
			if (currentmotion != arg0) {
				return;
			}
			jointPos = lbr.getCurrentJointPosition();
			setisLBRMoving(false);
			setLBRMotionActive(false);
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

	public class JointPositionMotionListener implements IMotionContainerListener {
		@Override
		public void onStateChanged(IExecutionContainer arg0, ExecutionState arg1) {
		}

		@Override
		public void containerFinished(IMotionContainer arg0) {
			if (currentmotion == arg0) {
				jointPos = lbr.getCurrentJointPosition();
				setisLBRMoving(false);
				setLBRMotionActive(false);
			}
		}

		@Override
		public void motionFinished(IMotion arg0) {
		}

		@Override
		public void motionStarted(IMotion arg0) {
		}
	}

	public class JointServoMotionListener implements IMotionContainerListener {
		@Override
		public void onStateChanged(IExecutionContainer arg0, ExecutionState arg1) {
		}

		@Override
		public void containerFinished(IMotionContainer arg0) {
			if (currentmotion == arg0) {
				jointServoRuntime = null;
				currentmotion = null;
				jointPos = lbr.getCurrentJointPosition();
				setisLBRMoving(false);
				setLBRMotionActive(false);
			}
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
			stopCurrentMotion();
			setLBRMotionActive(false);
		} catch(Exception e){
			System.out.println("No motion to end!");
		}

		try{
			this.socket.close();
		} catch(Exception e){
			System.out.println("Could not close LBR commander connection: " +e);
		}
		System.out.println("LBR command closed!");
	}
}