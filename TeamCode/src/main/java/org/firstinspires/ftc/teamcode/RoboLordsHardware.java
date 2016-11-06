package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class can be used to define all the specific hardware for a single robot.
 * <p/>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RoboLordsHardware {
    //    Motors
    public DcMotor leftMotor = null; //left_drive
    public DcMotor rightMotor = null; //right_drive
//    public DcMotor  armMotor    = null;

    //    Servos
    public Servo leftClaw = null;  //left_hand
    public Servo tubeServo = null; //tube_servo
    public Servo rightClaw = null; //right_hand

    //    Sensors
    public OpticalDistanceSensor opticalDistanceSensor = null;
    public IrSeekerSensor irSeekerSensor = null;
    public TouchSensor touchSensor = null;

    //    Initial configuration values
    public static final double TUBE_SERVO_POSITION_START = 0.0;
    public static final double TUBE_SERVO_POSITION_DEGREE_45 = 0.25;
    public static final double TUBE_SERVO_POSITION_DEGREE_90 = 0.5;
    public static final double TUBE_SERVO_POSITION_DEGREE_180 = 1.0;
    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RoboLordsHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");
//        armMotor    = hwMap.dcMotor.get("left_arm");
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
//        armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftClaw = hwMap.servo.get("left_hand");
        tubeServo = hwMap.servo.get("tube_servo");
        rightClaw = hwMap.servo.get("right_hand");
        leftClaw.setPosition(MID_SERVO);
        tubeServo.setPosition(TUBE_SERVO_POSITION_START);
        rightClaw.setPosition(MID_SERVO);

//        Define and intialize the sensors
        opticalDistanceSensor = hwMap.opticalDistanceSensor.get("ods");
        irSeekerSensor = hwMap.irSeekerSensor.get("irs");
        touchSensor = hwMap.touchSensor.get("touch_sensor");
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
