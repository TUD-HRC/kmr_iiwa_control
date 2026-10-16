package kmr_ros2.comm;




public interface ISocket {
	
	public void close();
	public void send_message(String msg);
	public String receive_message();
	public byte[] encode(String string);
	public boolean isConnected();


}
