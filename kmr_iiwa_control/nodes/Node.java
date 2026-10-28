// Copyright 2019 Nina Marie Wahl og Charlotte Heggem.
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


import kmr_iiwa_control.comm.ISocket;
import kmr_iiwa_control.comm.TCPSocket;
import kmr_iiwa_control.comm.UDPSocket;


public abstract class Node extends Thread{
	
	// Runtime Variables
	private volatile static boolean shutdown;
	public volatile boolean closed = false;
	private static volatile boolean EmergencyStop;
	private volatile static boolean PathFinished = true;
	protected volatile static boolean isKMPmoving = false;
	private volatile static boolean isLBRmoving = false;
	private volatile static boolean isKMPconnected;
	private volatile static boolean isLBRconnected;

	// Socket
	protected ISocket socket;
	private String ConnectionType;
	private String ip;
	private int port;
	private boolean nodeThreadStarted;
	public static int connection_timeout = 5000;

	
	// For KMP sensor reader:
	protected ISocket laser_socket;
	protected ISocket odometry_socket;
	private String KMP_laser_ip;
	private String KMP_odom_ip;
	private int KMP_laser_port;
	private int KMP_odom_port;
	private String LaserConnectionType;
	private String OdometryConnectionType;
	
	protected String nodeName;
	
	public Node(String ip1, int port1, String Conn1, 
		String ip2, int port2, String Conn2, 
		String nodeName ){

		if(nodeName == null){
			System.out.println("This is a test, nodeName is null in Node.");
		}
		this.KMP_laser_ip = ip1;
		this.KMP_odom_ip = ip2;
		this.KMP_laser_port = port1;
		this.KMP_odom_port = port2;
		this.LaserConnectionType = Conn1;
		this.OdometryConnectionType = Conn2;
		this.nodeName = nodeName;
		setShutdown(false);
		
		createSocket("Laser");
		createSocket("Odom");
		
	}
	
	public Node(String ip, int port, String Conn, String nodeName) {
		this.ip = ip;
		this.ConnectionType = Conn;
		this.port = port;
		this.nodeName = nodeName;
		setShutdown(false);
		createSocket();
	}
	
	public void createSocket(){
		if (this.ConnectionType == "TCP") {
			 this.socket = new TCPSocket(this.nodeName, this.ip, this.port);
		}
		else {
			this.socket = new UDPSocket(this.port, this.nodeName);
		}
	}
	
	public void createSocket(String Type){
		if (Type=="Laser"){
			if (LaserConnectionType == "TCP") {
				this.laser_socket = new TCPSocket(this.nodeName, KMP_laser_ip, KMP_laser_port);
			} else {
				this.laser_socket = new UDPSocket(KMP_laser_port, this.nodeName);
			}
		} else if(Type=="Odom") {
			if(OdometryConnectionType == "TCP") {
				this.odometry_socket = new TCPSocket(this.nodeName, KMP_odom_ip, KMP_odom_port);

			} else {
				this.odometry_socket = new UDPSocket(KMP_odom_port, this.nodeName);
			}
		}
	}
	
	public boolean isSocketConnected() {
			return this.socket.isConnected();
	}
	
	public boolean isNodeRunning() {
		return this.isSocketConnected() && (!(this.closed) && !this.getShutdown());
	}
	
	public static void setEmergencyStop(boolean es){
		EmergencyStop = es;
	}
	
	public boolean getEmergencyStop(){
		return EmergencyStop;
	}
	
	public synchronized boolean startNode() {
		if (nodeThreadStarted || closed || !isSocketConnected()) {
			return false;
		}
		nodeThreadStarted = true;
		super.start();
		return true;
	}

	@Override
	public synchronized void start() {
		startNode();
	}
	
	public boolean getShutdown() {
		return shutdown;
	}
	
	public void setShutdown(boolean in) {
		System.out.println("shutdown set by " + this.nodeName + " to " + in);
		shutdown=in;
	}
	
	public abstract void run();
	
	public abstract void close();
	
	public boolean getisPathFinished() {
		return PathFinished;
	}
	
	public void setisPathFinished(boolean in) {
		PathFinished = in;
	}
	
	public boolean getisLBRMoving() {
		return isLBRmoving;
	}
	
	public void setisLBRMoving(boolean in) {
		isLBRmoving = in;
	}
	
	public boolean getisKMPMoving() {
		return isKMPmoving;
	}
	
	public void setisKMPMoving(boolean in) {
		isKMPmoving = in;
	}
	
	public boolean getisLBRConnected() {
		return isLBRconnected;
	}
	
	public void setisLBRConnected(boolean in) {
		isLBRconnected = in;
	}
	
	public boolean getisKMPConnected() {
		return isKMPconnected;
	}
	
	public void setisKMPConnected(boolean in) {
		isKMPconnected = in;
	}

}
