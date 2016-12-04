package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Blue:01: Press Beacon and Push Capball", group = "RoboLords")
public class AutoPressBeaconAndPushCapball extends RoboLordsLinearOpMode {
    private static final double DRIVE_SLOW_SPEED = 0.25;
    private static final double DRIVE_NORMAL_SPEED = 0.5;
    private static final double DRIVE_HIGH_SPEED = 1.0;
    private static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        double timeoutSeconds;

        robot.init(hardwareMap);

        log("Status", "Resetting Encoders");
        telemetry.update();
        resetEncoders();

        log("COUNTS_PER_INCH", "%7f", COUNTS_PER_INCH);
        // Send telemetry message to indicate successful Encoder reset
        log("Path0", "Starting at %7d :%7d",
                robot.leftDriveMotor.getCurrentPosition(),
                robot.rightDriveMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        timeoutSeconds = 25.0;
        runtime.reset();
        enableTouchSensor(true);
        enableColorSensor(true);
        encoderDrive(DRIVE_SLOW_SPEED, 125);
        while (opModeIsActive() && isRedLightDetected() && runtime.seconds() < timeoutSeconds) {
            sleep(1500);
            if (isBlueLightDetected()) {
                break;
            }
            enableTouchSensor(false);
            encoderDrive(DRIVE_SLOW_SPEED, -6);
            sleep(1000);
            enableTouchSensor(true);
            encoderDrive(DRIVE_NORMAL_SPEED, 10);
            //check if blue beacon is on. If not backup and press again after 5 seconds
        }

        enableTouchSensor(false);
        encoderDrive(DRIVE_SLOW_SPEED, -12);
        encoderDriveTurnLeft(TURN_SPEED);
//        encoderDriveTurnRight(TURN_SPEED);

//        encoderDrive(DRIVE_NORMAL_SPEED, -75, 75, 30.0);  // S2: Turn Left 12 Inches with 4 Sec timeout
//        idle();
//        encoderDrive(TURN_SPEED, 12, -12, 30.0);  // S2: Turn Left 12 Inches with 4 Sec timeout
//        idle();
//        enableOpticalDistanceSensor(true);
//        encoderDrive(DRIVE_NORMAL_SPEED, 48, 48, 50.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        if (robot.touchSensor.isPressed()) {
//            encoderDrive(DRIVE_SLOW_SPEED, -4, -4, 4.0);
//            encoderDrive(DRIVE_HIGH_SPEED, 6, 6, 4.0);
//        }
//        encoderDrive(DRIVE_SLOW_SPEED, -48, -48, 10.0);
//        encoderDrive(TURN_SPEED, -24, 24, 4.0);
//        disableAllSensors();
//        enableColorSensor(true);
//        encoderDrive(DRIVE_SLOW_SPEED, -36, -36, 10.0);

        //Any servo operations can go here
//        robot.leftClaw.setPosition(1.0);
//        robot.rightClaw.setPosition(0.0);
//        sleep(1000);     // pause for servos to move

//        log("Path", "Complete");
//        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
