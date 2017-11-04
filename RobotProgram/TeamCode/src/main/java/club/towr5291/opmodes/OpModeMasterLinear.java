package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * Created by ianhaden on 8/02/2017.
 */

public abstract class OpModeMasterLinear extends LinearOpMode implements TrcRobot.RobotMode {

    private static OpModeMasterLinear instance = null;
    private static String opModeName = null;
    private static final boolean debugEnabled = false;

    private final static String OPMODE_AUTO     = "Auto";
    private final static String OPMODE_TELEOP   = "TeleOp";
    private final static String OPMODE_TEST     = "Test";

    private final static long LOOP_PERIOD = 20;
    private static double opModeStartTime = 0.0;
    private static double opModeElapsedTime = 0.0;
    private static double loopStartTime = 0.0;
    private static long loopCounter = 0;

    boolean initialized = false;
    private TrcDbgTrace dbgTrace = null;

    private static TrcDbgTrace globalTracer = null;

    private TrcTaskMgr taskMgr;

    boolean isInitialized() {
        return initialized;
    }

    protected void initOpenCv()
    {
        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext)
        {
            @Override
            public void onManagerConnected(int status)
            {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        initialized = true;
                        RobotLog.i("OpMasterLinear", "OpenCV loaded successfully");
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };


        if (!OpenCVLoader.initDebug())
        {
            RobotLog.d("OpMasterLinear", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, hardwareMap.appContext,  mLoaderCallback);
            initialized = false;
        } else
        {
            initialized = true;
            RobotLog.d("OpMasterLinear", "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public OpModeMasterLinear()
    {
        super();

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace("OpModeMasterLinear", false,  TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
        }

        instance = this;
        //
        // Create task manager. There is only one global instance of task manager.
        //
        taskMgr = new TrcTaskMgr();
    }


    /**
     * This method returns the saved instance. This is a static method. So other class can get to this class instance
     * by calling getInstance(). This is very useful for other classes that need to access the public fields and
     * methods.
     *
     * @return save instance of this class.
     */
    public static OpModeMasterLinear getInstance()
    {
        if (instance == null) throw new NullPointerException("You are not using OpModeMasterLinear!");
        return instance;
    }   //getInstance


    /**
     * This method returns a global debug trace object for tracing OpMode code. If it doesn't exist yet, one is
     * created. This is an easy way to quickly get some debug output without a whole lot of setup overhead as the
     * full module-based debug tracing.
     *
     * @return global opMode trace object.
     */
    public static TrcDbgTrace getGlobalTracer()
    {
        if (globalTracer == null)
        {
            globalTracer = new TrcDbgTrace(opModeName != null? opModeName: "globalTracer", false,
                    TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
        }

        return globalTracer;
    }   //getGlobalTracer

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station phone is pressed.
     */
    public abstract void initRobot();

    /**
     * This method is called when our OpMode is loaded and the "Init" button on the Driver Station is pressed.
     */
    @Override
    public void runOpMode()
    {
        final String funcName = "runOpMode";
        HalDashboard dashboard = HalDashboard.createInstance(telemetry);
        TrcRobot.RunMode runMode;

        if (debugEnabled)
        {
            if (dbgTrace == null)
            {
                dbgTrace = new TrcDbgTrace("OpModeMasterLinear", false, TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
            }
        }

        //
        // Determine run mode. Note that it means the OpMode must have "Auto", "TeleOp" or "Test" in its name.
        //
        String opModeFullName = this.toString();
        opModeName = "Invalid";

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "opModeFullName=<%s>", opModeFullName);
        }

        if (opModeFullName.contains(OPMODE_AUTO))
        {
            runMode = TrcRobot.RunMode.AUTO_MODE;
            opModeName = "Auto";
        }
        else if (opModeFullName.contains(OPMODE_TELEOP))
        {
            runMode = TrcRobot.RunMode.TELEOP_MODE;
            opModeName = "TeleOp";
        }
        else if (opModeFullName.contains(OPMODE_TEST))
        {
            runMode = TrcRobot.RunMode.TEST_MODE;
            opModeName = "Test";
        }
        else
        {
            throw new IllegalStateException("Invalid OpMode (must be either Auto, TeleOp or Test.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "runMode=%s", runMode.toString());
        }

        //
        // robotInit contains code to initialize the robot.
        //
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running robotInit ...");
        }
        dashboard.displayPrintf(0, "initRobot starting...");
        initRobot();
        dashboard.displayPrintf(0, "initRobot completed!");

        //
        // Run initPeriodic while waiting for competition to start.
        //
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running initPeriodic ...");
        }
        loopCounter = 0;
        dashboard.displayPrintf(0, "initPeriodic starting...");
        while (!isStarted())
        {
            loopCounter++;
            loopStartTime = TrcUtil.getCurrentTime();
            initPeriodic();
        }
        dashboard.displayPrintf(0, "initPeriodic completed!");
        opModeStartTime = TrcUtil.getCurrentTime();

        //
        // Prepare for starting the run mode.
        //
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running Start Mode Tasks ...");
        }
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, runMode);

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running startMode ...");
        }
        startMode();

        long nextPeriodTime = TrcUtil.getCurrentTimeMillis();
        loopCounter = 0;
        while (opModeIsActive())
        {
            loopCounter++;
            loopStartTime = TrcUtil.getCurrentTime();
            opModeElapsedTime = loopStartTime - opModeStartTime;

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running PreContinuous Tasks ...");
            }
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, runMode);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running runContinuous ...");
            }
            runContinuous(opModeElapsedTime);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running PostContinuous Tasks ...");
            }
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, runMode);

            if (TrcUtil.getCurrentTimeMillis() >= nextPeriodTime)
            {
                dashboard.displayPrintf(0, "%s: %.3f", opModeName, opModeElapsedTime);
                nextPeriodTime += LOOP_PERIOD;

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running PrePeriodic Tasks ...");
                }
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, runMode);

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running runPeriodic ...");
                }
                runPeriodic(opModeElapsedTime);

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running PostPeriodic Tasks ...");
                }

                taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTPERIODIC_TASK, runMode);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running stopMode ...");
        }
        stopMode();

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running Stop Mode Tasks ...");
        }
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, runMode);
    }   //runOpMode
    /**
     * This method is called periodically after initRobot() is called but before competition starts. Typically,
     * you override this method and put code that will check and display robot status in this method. For example,
     * one may monitor the gyro heading in this method to make sure there is no major gyro drift before competition
     * starts. By default, this method is doing exactly what waitForStart() does.
     */
    public synchronized void initPeriodic()
    {
        try
        {
            this.wait();
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
    }   //initPeriodic

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station phone is pressed. Typically, you put code that will prepare the robot for start
     * of competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     */
    @Override
    public void startMode()
    {
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean up
     * here such as disabling the sampling of some sensors.
     */
    @Override
    public void stopMode()
    {
    }   //stopMode

    /**
     * This method is called periodically about 50 times a second. Typically, you put code that doesn't require
     * frequent update here. For example, TeleOp joystick code can be put here since human responses are considered
     * slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runPeriodic(double elapsedTime)
    {
    }   //runPeriodic

    /**
     * This method is called periodically as fast as the control system allows. Typically, you put code that requires
     * servicing at a higher frequency here. To make the robot as responsive and as accurate as possible especially
     * in autonomous mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runContinuous(double elapsedTime)
    {
    }   //runContinuous


    /**
     * This method returns the name of the active OpMode.
     *
     * @return active OpMode name.
     */
    public static String getOpModeName()
    {
        return opModeName;
    }   //getOpModeName


}
