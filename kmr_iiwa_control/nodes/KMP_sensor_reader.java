package kmr_iiwa_control.nodes;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import kmr_iiwa_control.comm.ISocket;

import com.kuka.nav.Pose;
import com.kuka.nav.TupleSE2;
import com.kuka.nav.command.RequestContainer;
import com.kuka.nav.provider.LaserConfig;
import com.kuka.nav.provider.LaserScan;
import com.kuka.nav.provider.LaserSnapshot;
import com.kuka.nav.request.provider.LaserSnapshotContainer;
import com.kuka.nav.request.provider.LaserSnapshotRequest;
import com.kuka.nav.robot.MobileRobot;

public class KMP_sensor_reader extends Node {

	private static final int SNAPSHOT_INTERVAL_MSEC = 100;
	private static final int REQUEST_TIMEOUT_SECONDS = 2;
	private static final int EXPECTED_LASER_COUNT = 2;
	private static final long ODOMETRY_INTERVAL_MSEC = 50;

	private final MobileRobot mobileRobot;
	private final Set<Integer> loggedLaserConfigs = new HashSet<Integer>();
	private volatile RequestContainer activeRequest;
	public volatile boolean waiting = false;

	public KMP_sensor_reader(int laserPort, int odometryPort,
			String laserConnectionType, String odometryConnectionType,
			MobileRobot robot) {
		super(laserPort, laserConnectionType, odometryPort,
				odometryConnectionType, "KMP sensor reader");

		if (robot == null) {
			throw new IllegalArgumentException("Navigation MobileRobot must not be null");
		}
		this.mobileRobot = robot;

		Thread connectionMonitor = new Thread(new ConnectionMonitor(),
				"KMP sensor connection monitor");
		connectionMonitor.setDaemon(true);
		connectionMonitor.start();
	}

	@Override
	public void run() {
		Thread odometryThread = new Thread(new OdometrySender(),
				"KMP odometry sender");
		odometryThread.setDaemon(true);
		odometryThread.start();

		while (!closed && !getShutdown()) {
			if (!waiting && isLaserSocketConnected()) {
				readAndForwardSnapshot();
			}
			sleepQuietly(SNAPSHOT_INTERVAL_MSEC);
		}
	}

	private void readAndForwardSnapshot() {
		LaserSnapshotContainer result = null;
		try {
			result = mobileRobot.executeAsync(
					new LaserSnapshotRequest(1, EXPECTED_LASER_COUNT));
			activeRequest = result;
			result.awaitFinished(REQUEST_TIMEOUT_SECONDS, TimeUnit.SECONDS);

			if (result.hasError()) {
				System.out.println("KMP laser snapshot request failed: "
						+ result.getError());
				return;
			}
			if (!result.isValid()) {
				System.out.println("KMP laser snapshot request returned invalid data");
				return;
			}

			LaserSnapshot snapshot = result.getLaserSnapshot();
			if (snapshot == null) {
				System.out.println("KMP laser snapshot request returned no snapshot");
				return;
			}

			Collection<LaserScan> scans = snapshot.getScans();
			if (scans == null || scans.isEmpty()) {
				System.out.println("KMP laser snapshot contained no scans");
				return;
			}

			for (LaserScan scan : scans) {
				if (scan == null) {
					continue;
				}
				//logLaserConfig(scan);
				sendLaserScan(scan);
			}
		} catch (TimeoutException e) {
			if (result != null) {
				result.cancel();
			}
			System.out.println("Timed out waiting for KMP laser snapshot: " + e);
		} catch (RuntimeException e) {
			System.out.println("Could not retrieve KMP laser snapshot: " + e);
		} finally {
			activeRequest = null;
		}
	}

	private void logLaserConfig(LaserScan scan) {
		if (loggedLaserConfigs.contains(scan.getLaserId())) {
			return;
		}
		LaserConfig config = scan.getConfig();
		if (config == null) {
			System.out.println("KMP laser " + scan.getLaserId()
					+ " configuration is unavailable");
			return;
		}

		System.out.println("KMP laser " + scan.getLaserId() + " config: "
				+ "typeId=" + config.getLaserTypeId()
				+ ", startAngle=" + config.getStartAngle()
				+ ", fov=" + config.getFov()
				+ ", angularResolution=" + config.getAngularResolution()
				+ ", maximumRange=" + config.getMaximumRange()
				+ ", accuracy=" + config.getAccuracy()
				+ ", remissionMode=" + config.getRemissionMode());
		loggedLaserConfigs.add(scan.getLaserId());
	}

	private void sendLaserScan(LaserScan scan) {
		ISocket laserSocket = laser_socket;
		if (laserSocket == null || !laserSocket.isConnected()) {
			return;
		}
		String message = ">laserScan " + scan.getTimestamp() + " "
				+ scan.getLaserId() + " " + scan.getRangesAsString();

		try {
			laserSocket.send_message(message);
		} catch (RuntimeException e) {
			System.out.println("Could not send KMP laser scan: " + e);
		}
	}

	private void sendOdometry() {
		ISocket odometrySocket = odometry_socket;
		if (odometrySocket == null || !odometrySocket.isConnected()) {
			return;
		}

		try {
			Pose robotPose = mobileRobot.getPose();
			TupleSE2 velocity = mobileRobot.getVelocity();
			if (robotPose == null || velocity == null) {
				System.out.println("KMP odometry state is unavailable");
				return;
			}

			String message = ">odometry " + System.currentTimeMillis() + " "
					+ robotPose.getX() + " " + robotPose.getY() + " "
					+ robotPose.getTheta() + " " + velocity.getX() + " "
					+ velocity.getY() + " " + velocity.getTheta();
			odometrySocket.send_message(message);
		} catch (RuntimeException e) {
			System.out.println("Could not send KMP odometry: " + e);
		}
	}

	private class OdometrySender implements Runnable {
		@Override
		public void run() {
			while (!closed && !getShutdown()) {
				if (!waiting) {
					sendOdometry();
				}
				sleepQuietly(ODOMETRY_INTERVAL_MSEC);
			}
		}
	}

	private boolean isLaserSocketConnected() {
		return laser_socket != null && laser_socket.isConnected();
	}

	private boolean isOdometrySocketConnected() {
		return odometry_socket != null && odometry_socket.isConnected();
	}

	@Override
	public boolean isSocketConnected() {
		return isLaserSocketConnected() || isOdometrySocketConnected();
	}

	private void sleepQuietly(long milliseconds) {
		try {
			Thread.sleep(milliseconds);
		} catch (InterruptedException e) {
			if (!closed) {
				Thread.currentThread().interrupt();
				closed = true;
				System.out.println("KMP sensor reader interrupted");
			}
		}
	}

	private class ConnectionMonitor implements Runnable {
		@Override
		public void run() {
			while (!closed) {
				if (!isLaserSocketConnected()) {
					createSocket("Laser");
				}
				if (!isOdometrySocketConnected()) {
					createSocket("Odom");
				}
				if (isSocketConnected()) {
					startNode();
				}
				sleepQuietly(connection_timeout);
			}
		}
	}

	@Override
	public void close() {
		closed = true;
		RequestContainer request = activeRequest;
		if (request != null && !request.isFinished()) {
			request.cancel();
		}
		closeSocket(laser_socket, "laser");
		closeSocket(odometry_socket, "odometry");
		System.out.println("KMP sensor reader closed");
	}

	private void closeSocket(ISocket socket, String name) {
		if (socket == null) {
			return;
		}
		try {
			socket.close();
		} catch (RuntimeException e) {
			System.out.println("Could not close KMP " + name + " connection: " + e);
		}
	}
}
