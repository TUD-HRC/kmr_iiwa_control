package kmr_iiwa_control.config;
public final class Config {

    public static final String NAV_ROBOT_NAME = "Nav_KMR_200_iiwa_14_R820_1";
    public static final String CONTROLLER_NAME = "KUKA_Sunrise_Cabinet_1";

    public static final String SERVER_IP = "172.31.1.149";

    public static final int KMP_STATUS_PORT   = 30001;
    public static final int KMP_COMMAND_PORT  = 30002;
    public static final int KMP_LASER_PORT    = 30003;
    public static final int KMP_ODOM_PORT     = 30004;

    public static final int LBR_COMMAND_PORT  = 30005;
    public static final int LBR_STATUS_PORT   = 30006;
    public static final int LBR_SENSOR_PORT   = 30007;
    // Move the arm to /DrivePos during initialization.
    public static final boolean LBR_MOVE_TO_DRIVE_POS_ON_START = true;

    public static final String CONNECTION_TYPE = "UDP";

    // ── Feature switches ─────────────────────────────────────────
    // Set a flag to false to keep that functionality from being
    // constructed, connected or started. Changes require a rebuild
    // and redeploy through Sunrise Workbench.
    public static final boolean ENABLE_KMP_COMMANDS = true;
    public static final boolean ENABLE_KMP_STATUS   = false;
    public static final boolean ENABLE_KMP_SCAN     = true;
    // Per-laser forwarding. A KUKA snapshot still contains every laser;
    // these only decide which scans are sent to ROS over UDP.
    public static final boolean ENABLE_KMP_SCAN_FRONT_RIGHT = true;
    public static final boolean ENABLE_KMP_SCAN_BACK_LEFT   = true;
    public static final boolean ENABLE_KMP_ODOMETRY = true;

    public static final boolean ENABLE_LBR_COMMANDS = true;
    public static final boolean ENABLE_LBR_SENSOR   = true;
    public static final boolean ENABLE_LBR_STATUS   = false;

    //KMP sensor
    public static final long KMP_SENSOR_SCAN_PERIOD_MSEC = 100;
    public static final long KMP_SENSOR_ODOM_PERIOD_MSEC = 200;
	public static final int REQUEST_TIMEOUT_SECONDS = 2;
	public static final int EXPECTED_LASER_COUNT = 2;

    //KMP status
    public static final long KMP_STATUS_PERIOD_MSEC = 30;

    //LBR sensor
    public static final int KMP_LASER_ID_FRONT_RIGHT = 1801;
    public static final int KMP_LASER_ID_BACK_LEFT   = 1802;
    public final static int LBR_SENSOR_PERIOD_MSEC = 10;
    public static final boolean ENABLE_KMP_SENSOR = ENABLE_KMP_SCAN || ENABLE_KMP_ODOMETRY;

    //LBR status
    public final static long LBR_STATUS_PERIOD_MSEC = 30;

    //LBR commander
    // Hardware safety handles the real stop; this only polls the
    // software flag, so a few milliseconds of latency are fine.
    public final static long LBR_COMMANDER_EMERGENCY_MONITOR_PERIOD_MSEC = 10;

    //KMP commander
    public final static long KMP_COMMANDER_EMERGENCY_MONITOR_PERIOD_MSEC = 10;

    private Config() {}
}