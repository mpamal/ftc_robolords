package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Blue:01: Press Beacon and Push Capball", group = "RoboLords")
public class AutoBlue01PressBeaconAndPushCapball extends RoboLordsLinearOpMode {
    private static final double DRIVE_SLOW_SPEED = 0.25;
    private static final double DRIVE_NORMAL_SPEED = 0.6;
    private static final double DRIVE_HIGH_SPEED = 1.0;
    private static final double TURN_SPEED = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        double timeoutSeconds;

        robot.init(hardwareMap);

//        log("COUNTS_PER_INCH", "%7f", COUNTS_PER_INCH);
        telemetry.setAutoClear(false);
        Telemetry.Line line = telemetry.addLine("Path");

        log("Path0", "Starting at %7d :%7d",
                robot.leftDriveMotor.getCurrentPosition(),
                robot.rightDriveMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        timeoutSeconds = 20.0;
        runtime.reset();
        disableAllSensors();
        enableOpticalDistanceSensor(true);
        enableTouchSensor(true);
        encoderDrive(DRIVE_NORMAL_SPEED, 30);
        encoderDriveTurnRight(TURN_SPEED);
        encoderDrive(DRIVE_NORMAL_SPEED, 12);
        encoderDriveTurnLeft(TURN_SPEED);
        encoderDrive(DRIVE_NORMAL_SPEED, 12);
        encoderDriveTurnRight(TURN_SPEED);
        enableColorSensors(true);
        encoderDrive(DRIVE_NORMAL_SPEED, 30);
        while (opModeIsActive() && !isRedOrBlueColorDetected() && runtime.seconds() < timeoutSeconds){
            disableAllSensors();
            encoderDrive(DRIVE_SLOW_SPEED, -6);
            moveLeft(DRIVE_SLOW_SPEED);
            if (opModeIsActive() && !isObstacleDetected()) {
                enableTouchSensor(true);
                enableOpticalDistanceSensor(true);
                enableColorSensors(true);
                encoderDrive(DRIVE_SLOW_SPEED, 20);
            }
            enableColorSensors(true);
            line.addData("Moving left, Red or blue detected:", isRedOrBlueColorDetected());
            telemetry.update();
        }

        line.addData("Beacon detected red or blue:", isRedOrBlueColorDetected());
        telemetry.update();

        disableAllSensors();
        enableTouchSensor(true);
        while (opModeIsActive() && isRedLightDetected() && runtime.seconds() < timeoutSeconds) {
            sleep(1000);
            if (isBlueLightDetected()) {
                break;
            }
            enableTouchSensor(false);
            encoderDrive(DRIVE_NORMAL_SPEED, -6);
            sleep(1000);
            enableTouchSensor(true);
            encoderDrive(DRIVE_NORMAL_SPEED, 20);
            //check if blue beacon is on. If not backup and press again after 5 seconds
        }
        line.addData("Beacon detected blue:", isBlueLightDetected());
        telemetry.update();

        disableAllSensors();
        //backup and push capball
        encoderDrive(DRIVE_NORMAL_SPEED, -12);
        encoderDriveTurnLeft(TURN_SPEED);
        encoderDriveTurnLeft(TURN_SPEED);
        encoderDriveTurn45Left(TURN_SPEED);
        enableTouchSensor(true);
        encoderDrive(DRIVE_NORMAL_SPEED, 34); //should be 60
        disableAllSensors();
        idle();
    }
}
