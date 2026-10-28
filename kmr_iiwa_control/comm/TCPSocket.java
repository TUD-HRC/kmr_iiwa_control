package kmr_iiwa_control.comm;


import java.io.IOException;

import java.net.DatagramPacket;

import java.nio.charset.Charset;

import kmr_iiwa_control.comm.ISocket;

import java.net.Socket;
    
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;

public class TCPSocket implements ISocket{
	
	
	public volatile boolean isConnected;
	public Socket TCPConn;
	DatagramPacket package_out;
	DatagramPacket package_in;
	public PrintWriter outputStream;
	public BufferedReader inputStream;
	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");
	private String ip;
	private int port;
	String nodeName;
	
	public TCPSocket(String nodeName, String ip, int port) {
		isConnected = false;
		this.nodeName = nodeName;
		this.ip = ip;
		this.port = port;
		TCPConn = connect();
	}
	

	public Socket connect()
	{
		while (true) {
			try {
				TCPConn = new Socket(this.ip, this.port);
				TCPConn.setReuseAddress(true);
				System.out.println(this.nodeName + " connecting TCP at " + this.ip + ":" + this.port);
				break;
			}
			catch(IOException err){
				System.out.println("ERROR: " + this.nodeName + " can not connect over TCP to : " + this.ip + ":" + this.port + ". Error: " + err);
				return null;
			}
		}

		try {
			outputStream = new PrintWriter(TCPConn.getOutputStream(),true);
			inputStream = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
			isConnected=true;
		} catch(Exception err){
			System.out.println("Error creating I/O ports for TCP communication for  "+ this.nodeName+ " at " + this.ip + ":" + this.port + ". Error: " + err);
			return null;
		}
		
		return TCPConn;

	}
	

	public void send_message(String buffer){
		int len = (this.encode(buffer)).length;
		String send_string = String.format("%010d", len) + " " + buffer;
		outputStream.write(send_string);
		outputStream.flush();
	}
	

	@Override
	public String receive_message(){
		String line;
		try {
			while(!this.inputStream.ready()){}
			line=this.inputStream.readLine();
			return line;
		
		} catch(Exception e){
				System.out.println(this.nodeName+ " could not receive message from TCP connection on port: "+ this.port + " Error: " +e);
				return "error";
		}
	}	

		        
	public void close(){
		try {
			TCPConn.close();
			System.out.println("TCP connection to ROS closed port: " + this.port);
			isConnected=false;
		} catch (Exception e) {
			System.out.println("ERROR closing the TCP communication of  "+ this.nodeName + "  on port: " + this.port + " error: " + e);
		}
	}
	

	public String decode(byte[] data) {
		String message = new String(data, 0, data.length, UTF8_CHARSET);
        return message;
	}

	@Override
	public byte[] encode(String string) {
		return string.getBytes(UTF8_CHARSET);
	}

	@Override
	public boolean isConnected() {
		return this.isConnected;
	}

}
