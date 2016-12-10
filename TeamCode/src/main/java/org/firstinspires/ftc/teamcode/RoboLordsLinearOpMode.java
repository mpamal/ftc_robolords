package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class RoboLordsLinearOpMode extends LinearOpMode {
    protected RoboLordsHardware robot = new RoboLordsHardware();
    protected ElapsedTime runtime = new ElapsedTime();

    //drive motor settings
    private static final double COUNTS_PER_MOTOR_REV = 1120;  //For AndyMark
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    protected static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    protected static double TURN_90_DEGREES_DISTANCE = 10;
    protected static double TURN_45_DEGREES_DISTANCE = 5;
    protected static double TURN_TO_MOVE_LEFT_OR_RIGHT = 5;

    //color sensor settings
    private static int COLOR_DETECTION_INTENSITY = 10;

    private boolean useTouchSensor = false;
    private boolean useColorSensor = false;
    private boolean useBlueColorSensor = false;
    private boolean useRedColorSensor = false;
    //    private boolean useRightColorSensor = false;
    private boolean useOpticalDistanceSensor = false;


    protected void encoderDrive(double speed,
                                double leftInches, double rightInches,
                                double timeoutSeconds) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
            driveForward(speed);

//            log("DrivePath", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//            log("DrivePath", "Current pos %7d :%7d", robot.leftDriveMotor.getCurrentPosition(), robot.rightDriveMotor.getCurrentPosition());

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

                if (useColorSensor) {
                    if (isRedOrBlueColorDetected()) {
                        log("Color", "Red or Blue. Now stopping robot");
                        break;
                    }
                }

                if (useOpticalDistanceSensor && isODSDetected()) {
                    log("ODS", "Obstacle Detected. Stopping robot");
                    break;
                }

//                log("left motor position:", "%7d", robot.leftDriveMotor.getCurrentPosition());
//                log("right motor position:", "%7d", robot.rightDriveMotor.getCurrentPosition());
                double leftDrivePosition = robot.leftDriveMotor.getCurrentPosition() / COUNTS_PER_INCH;
                double rightDrivePosition = robot.rightDriveMotor.getCurrentPosition() / COUNTS_PER_INCH;

//                log("DrivePath", "Current left: %7f, right: %7f", leftDrivePosition, rightDrivePosition);
//                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

//            log("DrivePath", "Reached to %7d :%7d", robot.leftDriveMotor.getCurrentPosition(), robot.rightDriveMotor.getCurrentPosition());
//            log("Runtime secs:", "%7f", runtime.seconds());
//            telemetry.update();

            stopDriving();

            // Turn off RUN_TO_POSITION
            robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);   // optional pause after each move
        }
    }

    protected void encoderDrive(double speed, double inches) throws InterruptedException {
        encoderDrive(speed, inches, inches, 30.0);
    }

    protected void encoderDriveTurnLeft(double speed) throws InterruptedException {
        encoderDrive(speed, -TURN_90_DEGREES_DISTANCE, TURN_90_DEGREES_DISTANCE, 30.0);
    }

    protected void encoderDriveTurnRight(double speed) throws InterruptedException {
        encoderDrive(speed, TURN_90_DEGREES_DISTANCE, -TURN_90_DEGREES_DISTANCE, 30.0);
    }


    protected void encoderDriveTurn45Left(double speed) throws InterruptedException {
        encoderDrive(speed, -TURN_45_DEGREES_DISTANCE, TURN_45_DEGREES_DISTANCE, 30.0);
    }

    protected void encoderDriveTurn45Right(double speed) throws InterruptedException {
        encoderDrive(speed, TURN_45_DEGREES_DISTANCE, -TURN_45_DEGREES_DISTANCE, 30.0);
    }

    protected void moveLeft(double speed) throws InterruptedException {
        encoderDrive(speed, TURN_TO_MOVE_LEFT_OR_RIGHT, -TURN_TO_MOVE_LEFT_OR_RIGHT, 30.0);
        encoderDrive(speed, -6);
        encoderDrive(speed, -TURN_TO_MOVE_LEFT_OR_RIGHT, TURN_TO_MOVE_LEFT_OR_RIGHT, 30.0);
        encoderDrive(speed, 6);
    }

    protected void moveRight(double speed) throws InterruptedException {
        encoderDrive(speed, -TURN_TO_MOVE_LEFT_OR_RIGHT, TURN_TO_MOVE_LEFT_OR_RIGHT, 30.0);
        encoderDrive(speed, -6);
        encoderDrive(speed, TURN_TO_MOVE_LEFT_OR_RIGHT, -TURN_TO_MOVE_LEFT_OR_RIGHT, 30.0);
        encoderDrive(speed, 6);
    }

    protected void driveForward(double power) {
        robot.leftDriveMotor.setPower(Math.abs(power));
        robot.rightDriveMotor.setPower(Math.abs(power));
    }

    protected void stopDriving() {
        robot.leftDriveMotor.setPower(0);
        robot.rightDriveMotor.setPower(0);
    }

    protected void log(String caption, String format, Object... args) {
        String logMsg = caption + ":" + String.format(format, args);
        Log.v("ROBOLORDS", logMsg);
        telemetry.addData(caption, format, args);
    }

    protected void log(String caption, Object msg) {
        String logMsg = caption + ":" + msg;
        Log.v("ROBOLORDS", logMsg);
        telemetry.addData(caption, msg);
    }

    protected void resetEncoders() throws InterruptedException {
        robot.leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void launchParticles() {
        ElapsedTime launchTime = new ElapsedTime();
        double timeout = 3.0;
        robot.launchMotor1.setPower(RoboLordsHardware.MOTOR_FULL_POWER_FORWARD);
        robot.launchMotor2.setPower(RoboLordsHardware.MOTOR_FULL_POWER_REVERSE);
        liftParticleBasket();
        while (launchTime.seconds() < timeout) {
            idle();
        }
//        sleep(3000);

        robot.launchMotor1.setPower(0);
        robot.launchMotor2.setPower(0);
        dropParticleBasket();
//        sleep(3000);
        launchTime.reset();
        while (launchTime.seconds() < timeout) {
            idle();
        }
        stopBasket();
    }

//    protected void launchParticles() {
//        robot.launchMotor1.setPower(RoboLordsHardware.MOTOR_FULL_POWER_FORWARD);
//        robot.launchMotor2.setPower(RoboLordsHardware.MOTOR_FULL_POWER_REVERSE);
//        ElapsedTime launchTime = new ElapsedTime();
//        double liftBasketTimeout = 3.0;
//        while (launchTime.seconds() < liftBasketTimeout) {
//            liftParticleBasket();
//        }
//        stopBasket();
//    }

    protected void pickupParticles() {
        robot.pickupMotor1.setPower(RoboLordsHardware.MOTOR_FULL_POWER_FORWARD);
//        robot.pickupMotor2.setPower(RoboLordsHardware.MOTOR_FULL_POWER_REVERSE);
    }

    protected void liftParticleBasket() {
        robot.liftMotor.setPower(RoboLordsHardware.LIFT_MOTOR_FORWARD);
    }

    protected void dropParticleBasket() {
        robot.liftMotor.setPower(RoboLordsHardware.LIFT_MOTOR_REVERSE);
    }

    protected void stopBasket() {
        robot.liftMotor.setPower(0);
    }


    protected void stopPickup() {
        robot.pickupMotor1.setPower(0);
//        robot.pickupMotor2.setPower(0);
    }

    protected void stopLaunching() {
        robot.launchMotor1.setPower(0);
        robot.launchMotor2.setPower(0);
    }

    protected void disableAllSensors() {
        useTouchSensor = false;
        useColorSensor = false;
        useBlueColorSensor = false;
        useRedColorSensor = false;
        useOpticalDistanceSensor = false;
        robot.leftColorSensor.enableLed(false);
//        robot.rightColorSensor.enableLed(false);
    }

    protected void enableTouchSensor(boolean isSensorEnabled) {
        this.useTouchSensor = isSensorEnabled;
    }

    protected boolean isTouchSensorPressed() {
        return robot.touchSensor.isPressed();
    }

    protected void enableColorSensors(boolean isSensorEnabled) {
        enableColorSensor1(true);
        enableColorSensor2(true);
    }

    protected void enableBlueColorSensor(boolean isSensorEnabled) {
        enableColorSensor1(true);
        enableColorSensor2(true);
    }

    private void enableColorSensor1(boolean isSensorEnabled) {
        robot.leftColorSensor.enableLed(isSensorEnabled);
        this.useColorSensor = isSensorEnabled;
    }

    private void enableColorSensor2(boolean isSensorEnabled) {
//        robot.rightColorSensor.enableLed(isSensorEnabled);
        this.useColorSensor = isSensorEnabled;
    }

    protected void enableOpticalDistanceSensor(boolean isSensorEnabled) {
        this.useOpticalDistanceSensor = isSensorEnabled;
    }

    protected boolean isWhiteLightDetected() {
        boolean isWhiteDetected = false;
        if (robot.leftColorSensor.red() > COLOR_DETECTION_INTENSITY
                && robot.leftColorSensor.green() > COLOR_DETECTION_INTENSITY
                && robot.leftColorSensor.blue() > COLOR_DETECTION_INTENSITY) {
            isWhiteDetected = true;
        }
//        if (robot.rightColorSensor.red() > COLOR_DETECTION_INTENSITY
//                && robot.rightColorSensor.green() > COLOR_DETECTION_INTENSITY
//                && robot.rightColorSensor.blue() > COLOR_DETECTION_INTENSITY) {
//            isWhiteDetected = true;
//        }
        // send the info back to driver station using telemetry function.
//        log("Clear", robot.leftColorSensor.alpha());
//        log("Red  ", robot.leftColorSensor.red());
//        log("Green", robot.leftColorSensor.green());
//        log("Blue ", robot.leftColorSensor.blue());
//
//        log("Clear", robot.rightColorSensor.alpha());
//        log("Red  ", robot.rightColorSensor.red());
//        log("Green", robot.rightColorSensor.green());
//        log("Blue ", robot.rightColorSensor.blue());
        return isWhiteDetected;
    }

    protected boolean isBlueLightDetected() {
        return isColorLightDetected("BLUE");
    }

    protected boolean isRedLightDetected() {
        return isColorLightDetected("RED");
    }

//    protected boolean isBlueOrRedLightDetected() {
//        return isColorLightDetected("BLUE") || isColorLightDetected("RED");
//    }

    private boolean isColorLightDetected(String colorName) {
        boolean isColorDetected = false;
//        if (colorName.equals("RED") && (robot.leftColorSensor.red() > COLOR_DETECTION_INTENSITY
//                && robot.rightColorSensor.red() > COLOR_DETECTION_INTENSITY)) {
//            isColorDetected = true;
//        } else if (colorName.equals("BLUE") && (robot.leftColorSensor.blue() > COLOR_DETECTION_INTENSITY
//                && robot.rightColorSensor.blue() > COLOR_DETECTION_INTENSITY)) {
//            isColorDetected = true;
//        }

        if (colorName.equals("RED") && (robot.leftColorSensor.red() > COLOR_DETECTION_INTENSITY)) {
            isColorDetected = true;
        } else if (colorName.equals("BLUE") && (robot.leftColorSensor.blue() > COLOR_DETECTION_INTENSITY)) {
            isColorDetected = true;
        }

        // send the info back to driver station using telemetry function.
//        log("Clear", robot.leftColorSensor.alpha());
//        log("Red  ", robot.leftColorSensor.red());
//        log("Green", robot.leftColorSensor.green());
//        log("Blue ", robot.leftColorSensor.blue());
//
//        log("Clear", robot.rightColorSensor.alpha());
//        log("Red  ", robot.rightColorSensor.red());
//        log("Green", robot.rightColorSensor.green());
//        log("Blue ", robot.rightColorSensor.blue());
        return isColorDetected;

    }

    protected boolean isRedOrBlueColorDetected() {
        boolean isColorDetected = false;
        if (robot.leftColorSensor.red() > COLOR_DETECTION_INTENSITY
                || robot.leftColorSensor.blue() > COLOR_DETECTION_INTENSITY ) {
            isColorDetected = true;
        }
//        if (robot.leftColorSensor.red() > COLOR_DETECTION_INTENSITY
//                || robot.rightColorSensor.red() > COLOR_DETECTION_INTENSITY
//                || robot.leftColorSensor.blue() > COLOR_DETECTION_INTENSITY
//                || robot.rightColorSensor.blue() > COLOR_DETECTION_INTENSITY) {
//            isColorDetected = true;
//        }
        // send the info back to driver station using telemetry function.
        log("Clear", robot.leftColorSensor.alpha());
        log("Red  ", robot.leftColorSensor.red());
        log("Green", robot.leftColorSensor.green());
        log("Blue ", robot.leftColorSensor.blue());

//        log("Clear", robot.rightColorSensor.alpha());
//        log("Red  ", robot.rightColorSensor.red());
//        log("Green", robot.rightColorSensor.green());
//        log("Blue ", robot.rightColorSensor.blue());
        return isColorDetected;

    }

    protected boolean isObstacleDetected() {
        return isTouchSensorPressed() || isODSDetected();
    }

    protected boolean isODSDetected() {
        return robot.opticalDistanceSensor.getRawLightDetected() > 0.7;
    }
}
