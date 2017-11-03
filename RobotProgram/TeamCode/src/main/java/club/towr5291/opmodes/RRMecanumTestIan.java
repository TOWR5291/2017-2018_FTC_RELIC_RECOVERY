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
public class RRMecanumTestIan extends OpMode {

    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "BaseDriveTest";

    //motors
    // Declare OpMode members.
    private HardwareDriveMotors robotDrive      = new HardwareDriveMotors();   // Use a Pushbot's hardware

    //mode selection stuff
    public boolean activated = false;
    public int mode = 0;

    //general variables
    public float speed = 0;
    public float turn = 0;
    public float intendedTurn = 0;
    public float strafe = 0;

    //all modes variables
    public double leftFrontSpeed = 0;
    public double leftBackSpeed = 0;
    public double rightFrontSpeed = 0;
    public double rightBackSpeed = 0;


    //gyro assisted and field-centric driving variables
    public int quadrant = 1;
    public double radius = 0;
    public double heading = 0;
    public float ajustedHeading = 0;
    public float revHeading = 0;
    public float driftCorrectionHeadingStart = 0;
    public boolean enableDriftCorrection = false;

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

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void init() {

        if (debug >= 1)
        {
            fileLogger = new FileLogger(runtime);
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
            Log.d(TAG, "Loaded sharePreferences");
            fileLogger.writeEvent(TAG, "Loaded sharePreferences");
            Log.d(TAG, "Loading baseHardware");
            fileLogger.writeEvent(TAG, "Loading baseHardware");
        }

        robotDrive.init(hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig));

        if (debug >= 1)
        {
            Log.d(TAG, "Loaded baseHardware");
            fileLogger.writeEvent(TAG, "Loaded baseHardware");
            Log.d(TAG, "Setting setHardwareDriveRunWithoutEncoders");
            fileLogger.writeEvent(TAG, "Setting setHardwareDriveRunWithoutEncoders");
        }

        robotDrive.setHardwareDriveRunWithoutEncoders();

        if (debug >= 1)
        {
            Log.d(TAG, "Set setHardwareDriveRunWithoutEncoders");
            fileLogger.writeEvent(TAG, "Set setHardwareDriveRunWithoutEncoders");
        }
    }

    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }


    @Override
    public void loop() {

        leftFrontSpeed = -gamepad1.left_stick_y;
        leftBackSpeed = gamepad2.left_stick_y;
        rightFrontSpeed = gamepad1.right_stick_y;
        rightBackSpeed = -gamepad2.right_stick_y;
        setMotorPowers(leftFrontSpeed,leftBackSpeed,rightFrontSpeed,rightBackSpeed);

    }

    public void setMotorPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
        robotDrive.setHardwareDriveLeft1MotorPower(-leftFront);
        robotDrive.setHardwareDriveLeft2MotorPower(leftBack);
        robotDrive.setHardwareDriveRight1MotorPower(rightFront);
        robotDrive.setHardwareDriveRight2MotorPower(-rightBack);
    }

    public double determineHeading(float x) {
        return Math.asin(x/radius);
    }

    public float determineAjustedHeading() {
        // ajustedHeading = heading from driver - robot heading relative to driver
        ajustedHeading = (float) (heading - revHeading);
        return ajustedHeading;
    };

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
                robotDrive.setHardwareDrivePower(speed);
                break;
            case "Right":
                robotDrive.setHardwareDrivePower(-speed);
                break;
        }
    }

    public enum handed {

        LEFT ("Left"),
        RIGHT ("Right");
        private final String value;

        handed(String value) {
            this.value = value;
        }

        public String toString() {
            return value;
        }
    }

}

