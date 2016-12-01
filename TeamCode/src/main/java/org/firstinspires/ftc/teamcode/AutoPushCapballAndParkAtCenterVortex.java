package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Blue:01: Push Capball & Park At Center Vortex", group = "RoboLords")
public class AutoPushCapballAndParkAtCenterVortex extends RoboLordsLinearOpMode {
    private static final double DRIVE_SLOW_SPEED = 0.75;
    private static final double DRIVE_NORMAL_SPEED = 1.0;
    private static final double DRIVE_HIGH_SPEED = 2.0;
    private static final double TURN_SPEED = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        double leftPower;
        double rightPower;
        double max;

        robot.init(hardwareMap);

        log("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        log("COUNTS_PER_INCH", "%7f", COUNTS_PER_INCH);

        // Send telemetry message to indicate successful Encoder reset
        log("Path0", "Starting at %7d :%7d",
                robot.leftDriveMotor.getCurrentPosition(),
                robot.rightDriveMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SLOW_SPEED, 48, 48, 10.0);  // S1: Forward 48 Inches with 10 Sec timeout
        encoderDrive(TURN_SPEED, -24, 24, 4.0);  // S2: Turn Left 12 Inches with 4 Sec timeout
        enableOpticalDistanceSensor(true);
        encoderDrive(DRIVE_SLOW_SPEED, 36, 36, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        if (robot.touchSensor.isPressed()) {
            encoderDrive(DRIVE_SLOW_SPEED, -4, -4, 4.0);
            encoderDrive(DRIVE_HIGH_SPEED, 6, 6, 4.0);
        }
        encoderDrive(DRIVE_SLOW_SPEED, -48, -48, 10.0);
        encoderDrive(TURN_SPEED, -24, 24, 4.0);
        disableAllSensors();
        enableColorSensor(true);
        encoderDrive(DRIVE_SLOW_SPEED, -36, -36, 10.0);

        //Any servo operations can go here
//        robot.leftClaw.setPosition(1.0);
//        robot.rightClaw.setPosition(0.0);
//        sleep(1000);     // pause for servos to move

        log("Path", "Complete");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
