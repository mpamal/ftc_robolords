package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by RoboLords
 */
public abstract class RoboLordsLinearOpMode extends LinearOpMode {
    protected RoboLordsHardware robot = new RoboLordsHardware();
    protected ElapsedTime runtime = new ElapsedTime();

    //drive motor settings
    private static final double COUNTS_PER_MOTOR_REV = 420;
    private static final double DRIVE_GEAR_REDUCTION = 7.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    protected static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //color sensor settings
    private static int RED_DETECTION_INTENSITY = 10;
    private static int GREEN_DETECTION_INTENSITY = 10;
    private static int BLUE_DETECTION_INTENSITY = 10;

    private boolean useTouchSensor = false;
    private boolean useColorSensor = false;
    private boolean useOpticalDistanceSensor = false;

    protected void encoderDrive(double speed,
                                double leftInches, double rightInches,
                                double timeoutSeconds) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDriveMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDriveMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.leftDriveMotor.setTargetPosition(newLeftTarget);
            robot.rightDriveMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDriveMotor.setPower(Math.abs(speed));
            robot.rightDriveMotor.setPower(Math.abs(speed));

            log("DrivePath", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            log("DrivePath", "Current pos %7d :%7d", robot.leftDriveMotor.getCurrentPosition(), robot.rightDriveMotor.getCurrentPosition());

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSeconds) &&
                    (robot.leftDriveMotor.isBusy() && robot.rightDriveMotor.isBusy())
                    ) {

                // Display it for the driver.

//                log("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                log("Path2", "Running at %7d :%7d", leftMotorCurrentPosition, rightMotorCurrentPosition);
//                log("Runtime secs:time-", "%7f : %7f", runtime.seconds(), runtime.time());

                if (useTouchSensor && robot.touchSensor.isPressed()) {
                    log("Touch", "Is Pressed. Now stopping robot");
                    break;
                }

                if (useColorSensor && isWhiteLightDetected()) {
                    log("Color", "White line detected. Now stopping robot");
                    break;
                }

                if (useOpticalDistanceSensor && isObstacleDetected()) {
                    log("ODS", "Obstacle Detected. Stopping robot");
                    break;
                }

                log("left motor position:", "%7d", robot.leftDriveMotor.getCurrentPosition());
                log("right motor position:", "%7d", robot.rightDriveMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

//            log("DrivePath", "Reached to %7d :%7d", robot.leftDriveMotor.getCurrentPosition(), robot.rightDriveMotor.getCurrentPosition());
//            log("Runtime secs:", "%7f", runtime.seconds());
//            telemetry.update();

            // Stop all motion;
            robot.leftDriveMotor.setPower(0);
            robot.rightDriveMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    protected void log(String caption, String format, Object... args){
        String logMsg = caption + ":" + String.format(format, args);
        Log.v("ROBOLORDS", logMsg);
        telemetry.addData(caption, format, args);
    }

    protected void log(String caption, Object msg){
        String logMsg = caption + ":" + msg;
        Log.v("ROBOLORDS", logMsg);
        telemetry.addData(caption, msg);
    }

    protected void disableAllSensors() {
        useTouchSensor = false;
        useColorSensor = false;
        useOpticalDistanceSensor = false;
        robot.colorSensor.enableLed(false);
    }

    protected void enableTouchSensor(boolean isSensorEnabled) {
        this.useTouchSensor = isSensorEnabled;
    }

    protected void enableColorSensor(boolean isSensorEnabled) {
        robot.colorSensor.enableLed(isSensorEnabled);
        this.useColorSensor = isSensorEnabled;
    }

    protected void enableOpticalDistanceSensor(boolean isSensorEnabled) {
        this.useOpticalDistanceSensor = isSensorEnabled;
    }

    protected boolean isWhiteLightDetected() {
        boolean isWhiteDetected = false;
        if (robot.colorSensor.red() > RED_DETECTION_INTENSITY
                && robot.colorSensor.green() > GREEN_DETECTION_INTENSITY
                && robot.colorSensor.blue() > BLUE_DETECTION_INTENSITY) {
            isWhiteDetected = true;
        }
        // send the info back to driver station using telemetry function.
        log("Clear", robot.colorSensor.alpha());
        log("Red  ", robot.colorSensor.red());
        log("Green", robot.colorSensor.green());
        log("Blue ", robot.colorSensor.blue());
        return isWhiteDetected;
    }

    protected boolean isBlueLightDetected() {
        boolean isBlueDetected = false;
        if (robot.colorSensor.blue() > BLUE_DETECTION_INTENSITY) {
            isBlueDetected = true;
        }
        // send the info back to driver station using telemetry function.
        log("Clear", robot.colorSensor.alpha());
        log("Red  ", robot.colorSensor.red());
        log("Green", robot.colorSensor.green());
        log("Blue ", robot.colorSensor.blue());
        return isBlueDetected;
    }

    protected boolean isObstacleDetected() {
        return robot.opticalDistanceSensor.getRawLightDetected() > 1.2;
    }
}
