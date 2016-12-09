package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Straight Line", group = "RoboLords")
public class AutoStraightLine extends RoboLordsLinearOpMode {
    private static final double DRIVE_SLOW_SPEED = 0.25;
    private static final double DRIVE_NORMAL_SPEED = 0.5;
    private static final double DRIVE_HIGH_SPEED = 1.0;
    private static final double TURN_SPEED = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        double timeoutSeconds;

        robot.init(hardwareMap);

        log("Status", "Resetting Encoders");
        telemetry.update();
//        resetEncoders();

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
        enableColorSensor1(true);
        enableColorSensor2(true);
        encoderDrive(DRIVE_SLOW_SPEED, 36);
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
