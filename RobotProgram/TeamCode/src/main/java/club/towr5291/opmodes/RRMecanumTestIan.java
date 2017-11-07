package club.towr5291.opmodes;

/**
 * Created by kids on 9/30/2017 at 9:55 AM.
 */

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareDriveMotors;



//import club.towr5291.robotconfig.HardwareArmMotorsRR;

@TeleOp(name="RR Mecanum Test IAN", group="RRTest")
//@Disabled
public class RRMecanumTestIan extends OpModeMasterLinear
{

    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "BaseDriveTest";

    //motors
    // Declare OpMode members.
    private HardwareDriveMotors robotDrive      = new HardwareDriveMotors();   // Use a Pushbot's hardware

    //mode selection stuff
    public int mode = 0;

    //general variables
    public float speed = 0;
    public float turn = 0;
    public float intendedTurn = 0;
    public float strafe = 0;

    //all modes variables
    public double dblLeftMotor1;
    public double dblLeftMotor2;
    public double dblRightMotor1;
    public double dblRightMotor2;

    //gyro assisted and field-centric driving variables
    public int quadrant = 1;
    public double radius = 0;
    public double heading = 0;
    public float ajustedHeading = 0;
    public float revHeading = 0;

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private int delay;
    private String robotConfig;
    private ElapsedTime runtime = new ElapsedTime();

    //set up the variables for file logger and what level of debug we will log info at
    private FileLogger fileLogger;
    private int debug = 3;

    //servos
    // the servos are on the servo controller
    private final static double SERVOLIFTLEFTTOP_MIN_RANGE      = 0;
    private final static double SERVOLIFTLEFTTOP_MAX_RANGE      = 180;
    private final static double SERVOLIFTLEFTTOP_HOME           = 90;
    private final static double SERVOLIFTLEFTTOP_GLYPH_RELEASE  = 60;
    private final static double SERVOLIFTLEFTTOP_GLYPH_GRAB     = 30;

    private final static double SERVOLIFTRIGHTTOP_MIN_RANGE     = 0;
    private final static double SERVOLIFTRIGHTTOP_MAX_RANGE     = 180;
    private final static double SERVOLIFTRIGHTTOP_HOME          = 90;
    private final static double SERVOLIFTRIGHTTOP_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTRIGHTTOP_GLYPH_GRAB    = 30;

    private final static double SERVOLIFTLEFTBOT_MIN_RANGE      = 0;
    private final static double SERVOLIFTLEFTBOT_MAX_RANGE      = 180;
    private final static double SERVOLIFTLEFTBOT_HOME           = 90;
    private final static double SERVOLIFTLEFTBOT_GLYPH_RELEASE  = 60;
    private final static double SERVOLIFTLEFTBOT_GLYPH_GRAB     = 30;

    private final static double SERVOLIFTRIGHTBOT_MIN_RANGE     = 0;
    private final static double SERVOLIFTRIGHTBOT_MAX_RANGE     = 180;
    private final static double SERVOLIFTRIGHTBOT_HOME          = 90;
    private final static double SERVOLIFTRIGHTBOT_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTRIGHTBOT_GLYPH_GRAB    = 30;

    private final static double SERVOJEWELLEFT_MIN_RANGE        = 0;
    private final static double SERVOJEWELLEFT_MAX_RANGE        = 1.0;
    private final static double SERVOJEWELLEFT_HOME             = 147;
    private final static double SERVOJEWELRIGHT_MIN_RANGE       = 4;
    private final static double SERVOJEWELRIGHT_MAX_RANGE       = 1.0;
    private final static double SERVOJEWELRIGHT_HOME            = 150;

    private Servo servoGlyphGripTopLeft;
    private Servo servoGlyphGripBotLeft;
    private Servo servoGlyphGripTopRight;
    private Servo servoGlyphGripBotRight;
    private Servo servoJewelLeft;
    private Servo servoJewelRight;

    private DigitalChannel green1LedChannel;
    private DigitalChannel red1LedChannel;
    private DigitalChannel blue1LedChannel;
    private DigitalChannel green2LedChannel;
    private DigitalChannel red2LedChannel;
    private DigitalChannel blue2LedChannel;
    private DigitalChannel limitswitch1;  // Hardware Device Object
    private DigitalChannel limitswitch2;  // Hardware Device Object
    private final boolean LedOn = false;
    private boolean LedOff = true;

    private void LedState (boolean g1, boolean r1, boolean b1, boolean g2, boolean r2, boolean b2) {
        green1LedChannel.setState(g1);
        red1LedChannel.setState(r1);
        blue1LedChannel.setState(b1);
        green2LedChannel.setState(g2);
        red2LedChannel.setState(r2);
        blue2LedChannel.setState(b2);
    }

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException
    {

        if (debug >= 1)
        {
            //create logging based on initial settings, sharepreferences will adjust levels
            fileLogger = new FileLogger(runtime);
            fileLogger.setDebugLevel(debug);
            fileLogger.setLogdEnabled(true);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
            Log.d(TAG, "Log Started");
            runtime.reset();
            telemetry.addData("FileLogger: ", runtime.toString());
            telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
            Log.d(TAG, "Loading sharePreferences");
            fileLogger.writeEvent(TAG, "Loading sharePreferences");
        }

        //load menu settings and setup robot and debug level
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunnerMecanum2x40");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        if (debug >= 1)
        {
            //adjust debug level based on shared preferences
            fileLogger.setDebugLevel(debug);
            fileLogger.writeEvent(TAG, "Loaded sharePreferences");
            fileLogger.writeEvent(TAG, "Loading LED Settings");
        }

        // get a reference to a Modern Robotics DIM, and IO channels.
        green1LedChannel = hardwareMap.get(DigitalChannel.class, "green1");    //  Use generic form of device mapping
        red1LedChannel = hardwareMap.get(DigitalChannel.class, "red1");    //  Use generic form of device mapping
        blue1LedChannel = hardwareMap.get(DigitalChannel.class, "blue1");    //  Use generic form of device mapping
        green2LedChannel = hardwareMap.get(DigitalChannel.class, "green2");    //  Use generic form of device mapping
        red2LedChannel = hardwareMap.get(DigitalChannel.class, "red2");    //  Use generic form of device mapping
        blue2LedChannel = hardwareMap.get(DigitalChannel.class, "blue2");    //  Use generic form of device mapping
        green1LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        red1LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        blue1LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        green2LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        red2LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        blue2LedChannel.setMode(DigitalChannel.Mode.OUTPUT);

        LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Loaded LED Settings");
            fileLogger.writeEvent(TAG, "Loading baseHardware");
        }

        robotDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig));

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Loaded baseHardware");
            fileLogger.writeEvent(TAG, "Setting setHardwareDriveRunWithoutEncoders");
        }

        robotDrive.setHardwareDriveRunWithoutEncoders();

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Set setHardwareDriveRunWithoutEncoders");
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            dblLeftMotor1 = Range.clip(+gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -1.0, 1.0);
            dblLeftMotor2 = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -1.0, 1.0);
            dblRightMotor1 = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -1.0, 1.0);
            dblRightMotor2 = Range.clip(+gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -1.0, 1.0);

            robotDrive.setHardwareDrivePower(dblLeftMotor1, dblLeftMotor2, dblRightMotor1, dblRightMotor2);


        }
    }

    public static double scaleRange(double value, double lowSrcRange, double highSrcRange, double lowDstRange, double highDstRange)
    {
        return lowDstRange + (value - lowSrcRange)*(highDstRange - lowDstRange)/(highSrcRange - lowSrcRange);
    }   //scaleRange


    public double determineHeading(float x) {
        return Math.asin(x/radius);
    }

    public float determineAjustedHeading() {
        // ajustedHeading = heading from driver - robot heading relative to driver
        ajustedHeading = (float) (heading - revHeading);
        return ajustedHeading;
    }

    public float determineSpeed(float angle, float distance) {
        speed = (float) (distance/(-Math.sin(angle)));
        return speed;
}

    public float determineStrafe(float angle, float distance) {
        strafe = (float) (distance/(-Math.sin(angle)));
        return strafe;
    }

    public void strafe(handed direction, double speed) {

        switch (direction.toString()) {
            case "Left":
                dblLeftMotor1  =  speed;
                dblLeftMotor2  =  -speed;
                dblRightMotor1 =  -speed;
                dblRightMotor2 =  speed;
                robotDrive.setHardwareDrivePower(dblLeftMotor1, dblLeftMotor2, dblRightMotor1, dblRightMotor2);
                break;
            case "Right":
                dblLeftMotor1  =  -speed;
                dblLeftMotor2  =  speed;
                dblRightMotor1 =  speed;
                dblRightMotor2 =  -speed;
                robotDrive.setHardwareDrivePower(dblLeftMotor1, dblLeftMotor2, dblRightMotor1, dblRightMotor2);
                break;
        }
    }

    enum handed {

        LEFT("Left"),
        RIGHT("Right");
        private final String value;

        handed(String value) {
            this.value = value;
        }

        public String toString() {
            return value;
        }
    }
}

