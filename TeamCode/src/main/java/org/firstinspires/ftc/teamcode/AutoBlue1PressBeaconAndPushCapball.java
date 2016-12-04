package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Blue:01: Press Beacon and Push Capball", group = "RoboLords")
public class AutoBlue1PressBeaconAndPushCapball extends RoboLordsLinearOpMode {
    private static final double DRIVE_SLOW_SPEED = 0.25;
    private static final double DRIVE_NORMAL_SPEED = 0.5;
    private static final double DRIVE_HIGH_SPEED = 1.0;
    private static final double TURN_SPEED = 0.10;

    @Override
    public void runOpMode() throws InterruptedException {
        double timeoutSeconds;

        robot.init(hardwareMap);

//        log("COUNTS_PER_INCH", "%7f", COUNTS_PER_INCH);
        log("Path0", "Starting at %7d :%7d",
                robot.leftDriveMotor.getCurrentPosition(),
                robot.rightDriveMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        timeoutSeconds = 20.0;
        runtime.reset();
        enableTouchSensor(true);
        enableColorSensor(true);
        encoderDrive(DRIVE_SLOW_SPEED, 135);
        while (opModeIsActive() && isRedLightDetected() && runtime.seconds() < timeoutSeconds) {
            sleep(1500);
            if (isBlueLightDetected()) {
                break;
            }
            enableTouchSensor(false);
            encoderDrive(DRIVE_SLOW_SPEED, -6);
            sleep(1000);
            enableTouchSensor(true);
            encoderDrive(DRIVE_NORMAL_SPEED, 36);
            //check if blue beacon is on. If not backup and press again after 5 seconds
        }
        disableAllSensors();
        //backup and push capball
        encoderDrive(DRIVE_SLOW_SPEED, -12);
        encoderDriveTurnLeft(TURN_SPEED);
        encoderDriveTurnLeft(TURN_SPEED);
        enableTouchSensor(true);
        encoderDrive(DRIVE_SLOW_SPEED, 36); //should be 60
        enableTouchSensor(false);
        encoderDrive(DRIVE_SLOW_SPEED, -4);
        enableTouchSensor(true);
        encoderDrive(DRIVE_NORMAL_SPEED, 6);
        disableAllSensors();
        idle();
    }
}
