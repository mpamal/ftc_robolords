package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Red:04: Press Beacon and Push Capball", group = "RoboLords")
public class AutoRed04PressBeaconAndPushCapball extends RoboLordsLinearOpMode {
    private static final double DRIVE_SLOW_SPEED = 0.25;
    private static final double DRIVE_NORMAL_SPEED = 0.5;
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
        enableTouchSensor(true);
        enableOpticalDistanceSensor(true);
        encoderDrive(DRIVE_NORMAL_SPEED, 55);
        encoderDriveTurnLeft(TURN_SPEED);
        encoderDrive(DRIVE_NORMAL_SPEED, 45);
        disableAllSensors();
        enableOpticalDistanceSensor(true);
        enableTouchSensor(true);
        enableColorSensor(true);
        while (opModeIsActive() && !isBlueOrRedLightDetected() && runtime.seconds() < 18.0){
            enableTouchSensor(false);
            enableOpticalDistanceSensor(false);
            encoderDrive(DRIVE_SLOW_SPEED, -6);
            moveLeft(DRIVE_SLOW_SPEED);
            while (opModeIsActive() && !isObstacleDetected()) {
                enableTouchSensor(true);
                enableOpticalDistanceSensor(true);
                encoderDrive(DRIVE_SLOW_SPEED, 12);
            }

            line.addData("Moving left, Red or blue detected:", isBlueOrRedLightDetected());
            telemetry.update();
        }

        line.addData("Beacon detected red or blue:", isBlueOrRedLightDetected());
        telemetry.update();

        disableAllSensors();
        enableTouchSensor(true);
        encoderDrive(DRIVE_NORMAL_SPEED, 12);

        while (opModeIsActive() && isRedLightDetected() && runtime.seconds() < 23.0) {
            sleep(1000);
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
        encoderDrive(DRIVE_NORMAL_SPEED, -6);
        encoderDriveTurnRight(DRIVE_NORMAL_SPEED);
        encoderDriveTurnRight(DRIVE_NORMAL_SPEED);
        encoderDrive(DRIVE_HIGH_SPEED, 36);

        line.addData("Beacon detected blue:", isBlueLightDetected());
        telemetry.update();

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
