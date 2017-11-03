package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.libraries.robotConfigSettings;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the hardware for a drive base.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "leftmotor1"
 * Motor channel:  Left  drive motor:        "leftmotor2"
 * Motor channel:  Right drive motor:        "rightmotor1"
 * Motor channel:  Right drive motor:        "rightmotor2"
 */
public class HardwareDriveMotors
{
    /* Public OpMode members. */
    public DcMotor  baseMotor1  = null;
    public DcMotor  baseMotor2  = null;
    public DcMotor  baseMotor3  = null;
    public DcMotor  baseMotor4  = null;

    /* local OpMode members. */
    HardwareMap hwMap            =  null;
    private ElapsedTime period   = new ElapsedTime();

    /* Constructor */
    public HardwareDriveMotors(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, robotConfigSettings.robotConfigChoice baseConfig) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        baseMotor1  = hwMap.dcMotor.get("leftMotor1");
        baseMotor2  = hwMap.dcMotor.get("leftMotor2");
        baseMotor3  = hwMap.dcMotor.get("rightMotor1");
        baseMotor4  = hwMap.dcMotor.get("rightMotor2");

        setHardwareDriveDirections(baseConfig);

        // Set all motors to zero power
        setHardwareDrivePower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareDriveResetEncoders();

        setHardwareDriveRunUsingEncoders();
    }

    public void setHardwareDriveDirections(robotConfigSettings.robotConfigChoice baseConfig){

        switch (baseConfig) {
            case TileRunner2x20:
            case TileRunner2x40:
            case TileRunner2x60:
                baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                baseMotor4.setDirection(DcMotor.Direction.REVERSE);
                break;
            case TileRunnerMecanum2x40:
                baseMotor1.setDirection(DcMotor.Direction.REVERSE);
                baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                baseMotor4.setDirection(DcMotor.Direction.FORWARD);
                break;
            default:
                baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                baseMotor4.setDirection(DcMotor.Direction.REVERSE);
                break;
        }
    }

    public int getHardwareDriveIsBusy() {
        //this will return a bitmapped integer,
        // int 0  = 0000 is right2, right1, left2, left1
        // int 1  = 0001 is right2, right1, left2, left1 (1)
        // int 2  = 0010 is right2, right1, left2 (1), left1
        // int 3  = 0011 is right2, right1, left2 (1), left1 (1)
        // int 4  = 0100 is right2, right1 (1), left2, left1
        // int 5  = 0101 is right2, right1 (1), left2, left1 (1)
        // int 6  = 0110 is right2, right1 (1), left2 (1), left1
        // int 7  = 0111 is right2, right1 (1), left2 (1), left1 (1)
        // int 8  = 1000 is right2 (1), right1, left2, left1
        // int 9  = 1001 is right2 (1), right1, left2, left1 (1)
        // int 10 = 1010 is right2 (1), right1, left2 (1), left1
        // int 11 = 1011 is right2 (1), right1, left2 (1), left1 (1)
        // int 12 = 1100 is right2 (1), right1 (1), left2, left1
        // int 13 = 1101 is right2 (1), right1 (1), left2, left1 (1)
        // int 14 = 1101 is right2 (1), right1 (1), left2, left1 (1)
        // int 15 = 1110 is right2 (1), right1 (1), left2 (1), left1
        // int 16 = 1111 is right2 (1), right1 (1), left2 (1), left1 (1)

        int myInt1 = (baseMotor1.isBusy()) ? 1 : 0;
        int myInt2 = (baseMotor2.isBusy()) ? 1 : 0;
        int myInt3 = (baseMotor3.isBusy()) ? 1 : 0;
        int myInt4 = (baseMotor4.isBusy()) ? 1 : 0;

        return (myInt1) + (2 * myInt2) + (4 * myInt3)  + (8 * myInt4);

    }

    public class motorEncoderPositions {

        private int motor1;      //is the current encoder position or motor 1
        private int motor2;      //is the current encoder position or motor 2
        private int motor3;      //is the current encoder position or motor 3
        private int motor4;      //is the current encoder position or motor 4


        // Constructor
        public motorEncoderPositions()
        {
            this.motor1 = 0;
            this.motor2 = 0;
            this.motor3 = 0;
            this.motor4 = 0;
        }

        public void setMotor1EncoderValue (int value) {
            this.motor1 = value;
        }

        public void setMotor2EncoderValue (int value) {
            this.motor2 = value;
        }

        public void setMotor3EncoderValue (int value) {
            this.motor3 = value;
        }

        public void setMotor4EncoderValue (int value) {
            this.motor4 = value;
        }

        public int getMotor1EncoderValue () {
            return this.motor1;
        }

        public int getMotor2EncoderValue () {
            return this.motor2;
        }

        public int getMotor3EncoderValue () {
            return this.motor3;
        }

        public int getMotor4EncoderValue () {
            return this.motor4;
        }

    }

    public motorEncoderPositions getHardwareDriveEncoderPosition() {

        motorEncoderPositions positions = new motorEncoderPositions();
        positions.setMotor1EncoderValue(baseMotor1.getCurrentPosition());
        positions.setMotor2EncoderValue(baseMotor2.getCurrentPosition());
        positions.setMotor3EncoderValue(baseMotor3.getCurrentPosition());
        positions.setMotor4EncoderValue(baseMotor4.getCurrentPosition());

        return positions;
    }

    public void setHardwareDriveResetEncoders() {
        baseMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHardwareDriveRunUsingEncoders() {
        setHardwareDriveLeftRunUsingEncoders();
        setHardwareDriveRightRunUsingEncoders();
    }

    public void setHardwareDriveLeftRunUsingEncoders() {
        baseMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareDriveRightRunUsingEncoders() {
        baseMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareDriveRunWithoutEncoders() {
        baseMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHardwareDriveRunToPosition() {
        setHardwareDriveLeftRunToPosition();
        setHardwareDriveRightRunToPosition();
    }

    public void setHardwareDriveLeftRunToPosition() {
        baseMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHardwareDriveRightRunToPosition() {
        baseMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //set the drive motors power, both left and right
    public void setHardwareDrivePower (double power) {
        setHardwareDriveLeftMotorPower(power);
        setHardwareDriveRightMotorPower(power);
    }

    //set the left motors drive power
    public void setHardwareDriveLeftMotorPower (double power) {
        setHardwareDriveLeft1MotorPower(power);
        setHardwareDriveLeft2MotorPower(power);
    }

    //set the right drive motors power
    public void setHardwareDriveRightMotorPower (double power) {
        setHardwareDriveRight1MotorPower(power);
        setHardwareDriveRight2MotorPower(power);
    }

    public void setHardwareDriveLeft1MotorPower (double power) {
        baseMotor1.setPower(power);
    }

    public void setHardwareDriveLeft2MotorPower (double power) {
        baseMotor2.setPower(power);
    }

    public void setHardwareDriveRight1MotorPower (double power) {
        baseMotor3.setPower(power);
    }

    public void setHardwareDriveRight2MotorPower (double power) {
        baseMotor4.setPower(power);
    }



}

