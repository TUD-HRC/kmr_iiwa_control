package kmr_iiwa_control.comm;


import java.net.BindException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.charset.Charset;

import kmr_iiwa_control.comm.ISocket;

public class UDPSocket implements ISocket{
	
	public volatile boolean isConnected;
	DatagramSocket udpConn;
	DatagramPacket package_out;
	DatagramPacket package_in;
	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");
	private String ip;
	private int port;
	static BindException b;
	String nodeName;

	
	public UDPSocket(String nodeName, String ip, int port) {
		isConnected = false;
		this.nodeName=nodeName;
		this.ip = ip;
		this.port = port;
		this.udpConn = connect();
	}
	
	public DatagramSocket connect()
	{
		while (!isConnected){
			try {
				System.out.println(this.nodeName + " connecting UDP at " + this.ip + ":" + this.port);
		        InetSocketAddress socket_address = new InetSocketAddress(this.port);
		    	InetAddress address = InetAddress.getByName(this.ip);
		        
		        udpConn = new DatagramSocket(null); 
		        udpConn.setReuseAddress(true);
		        udpConn.bind(socket_address);
		        
		        byte buf[] = "HALLA ROS".getBytes();
		        byte buf1[] = new byte[1024]; 
		        
		        package_out = new DatagramPacket(buf, buf.length, address, this.port); 
		        package_in = new DatagramPacket(buf1, buf1.length); 
		      
		        // send() method 
		        udpConn.send(package_out); 
		  
		        // receive() method 
		        udpConn.setSoTimeout(3000); //ms

		        udpConn.receive(package_in); 
		        String s = decode(package_in);
		        System.out.println(this.nodeName + " received packet data over UDP on port : " + port + " Message: " +s);  
		        
		        if(s.length()<1){
					udpConn.close();
		       		isConnected=false;
		       		System.out.println( this.nodeName + "  did not receive any message in 3 seconds, shutting off");
		       		break;
				}
		        udpConn.setSoTimeout(0);
		        isConnected=true;
			}
			catch(Exception e1){
		        System.out.println("ERROR connecting  " + this.nodeName + " to ROS over UDP on port: " + this.port + " Error: " + e1);
		        isConnected=false;
		        close();
				b = new BindException();
		        if(e1.getClass().equals(b.getClass())){
		        	e1.printStackTrace();
		        	break;
		        }
		        	break;
		        }
			}
		return udpConn;
	}
	
	public String decode(DatagramPacket pack)  {
    	byte[] data = pack.getData();
    	String message = new String(data,0,pack.getLength(), UTF8_CHARSET);
        return message;

    }

	@Override
    public byte[] encode(String string) {
        return string.getBytes(UTF8_CHARSET);
    }
    
	@Override
    public void send_message(String msg)
	{
    	byte[] bytlist = msg.getBytes(UTF8_CHARSET);
    	package_out.setData(bytlist);
    	package_out.setLength(bytlist.length);
     try {
			udpConn.send(package_out);
		} catch (Exception e) {
			System.out.println( this.nodeName + " could not send package over UDP on port: "  + this.port + " error: " + e);
		}
	}
    @Override
	public String receive_message()
	{
		String line;
		try{
			udpConn.receive(package_in);
			line = decode(package_in);
	    	return line;
		}
		catch(Exception e){
			System.out.println("Error receiving package  " + this.nodeName + " over UDP on port: " + this.port + " error: " + e);
			return " ";
		}
	}
	
    @Override
	public void close(){
		try {
			udpConn.close();
			System.out.println("Connection to ROS closed on port: " + port);
			isConnected=false;
		} catch (Exception e) {
			System.out.println("ERROR closing the UDP communication for "+ this.nodeName + " on port to ROS: " + port);
		}
	}


	@Override
	public boolean isConnected() {	
		return this.isConnected;
	}

}
