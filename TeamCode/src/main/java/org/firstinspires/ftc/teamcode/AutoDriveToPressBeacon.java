package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Auto: Drive To Press Beacon", group = "RoboLords")
public class AutoDriveToPressBeacon extends LinearOpMode {
    RoboLordsHardware robot = new RoboLordsHardware();
    private ElapsedTime runtime = new ElapsedTime();

    //    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double COUNTS_PER_MOTOR_REV = 7;
    static final double DRIVE_GEAR_REDUCTION = 60.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;
    boolean currentButtonState = true;
    boolean previousState = true;

    @Override
    public void runOpMode() throws InterruptedException {
        double leftPower;
        double rightPower;
        double max;

        robot.init(hardwareMap);

        telemetry.addData("Status", "Waiting to start");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            // check the status of the x button on either gamepad.
            currentButtonState = gamepad1.x;

            // check for button state transitions.
            if ((currentButtonState == true) && (currentButtonState != previousState)) {
                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                robot.colorSensor1.enableLed(bLedOn);
            }
            if ((currentButtonState == true) && (currentButtonState != previousState)) {
                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                robot.colorSensor2.enableLed(bLedOn);
            }

            // update previous state variable.
            previousState = currentButtonState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(robot.colorSensor1.red() * 8, robot.colorSensor1.green() * 8, robot.colorSensor1.blue() * 8, hsvValues);
            Color.RGBToHSV(robot.colorSensor2.red() * 8, robot.colorSensor1.green() * 8, robot.colorSensor1.blue() * 8, hsvValues);

            Color.RGBToHSV(robot.colorSensor1.red() * 8, robot.colorSensor2.green() * 8, robot.colorSensor2.blue() * 8, hsvValues);
            Color.RGBToHSV(robot.colorSensor2.red() * 8, robot.colorSensor2.green() * 8, robot.colorSensor2.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
//            telemetry.addData("LED", bLedOn ? "On" : "Off");
//            telemetry.addData("Clear", robot.colorSensor.alpha());
//            telemetry.addData("Red  ", robot.colorSensor.red());
//            telemetry.addData("Green", robot.colorSensor.green());
//            telemetry.addData("Blue ", robot.colorSensor.blue());
//            telemetry.addData("Hue", hsvValues[0]);

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
