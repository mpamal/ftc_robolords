package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Auto: Test Color Sensor", group = "RoboLords")
public class AutoTestColorSensor extends RoboLordsLinearOpMode {
    private static final double DRIVE_SLOW_SPEED = 0.5;
    private static final double DRIVE_NORMAL_SPEED = 0.75;
    private static final double DRIVE_HIGH_SPEED = 1.0;
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

        telemetry.addData("COUNTS_PER_INCH", "%7f", COUNTS_PER_INCH);

        // Send telemetry message to indicate successful Encoder reset
        log("Path0", "Starting at %7d :%7d",
                robot.leftDriveMotor.getCurrentPosition(),
                robot.rightDriveMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        disableAllSensors();
        enableColorSensor(true);
        encoderDrive(DRIVE_SLOW_SPEED, 48, 48, 10.0);  // S1: Forward 48 Inches with 10 Sec timeout
        if (isBlueLightDetected() || isWhiteLightDetected()) {
            disableAllSensors();
//            encoderDrive(DRIVE_SLOW_SPEED, 12, 12, 4.0);
            encoderDrive(TURN_SPEED, -24, 24, 4.0);
            enableTouchSensor(true);
            encoderDrive(DRIVE_NORMAL_SPEED, 24, 24, 4.0);
            //check if blue beacon is on. If not backup and press again after 5 seconds
        }
        disableAllSensors();

        //Any servo operations can go here
//        robot.leftClaw.setPosition(1.0);
//        robot.rightClaw.setPosition(0.0);
//        telemetry.addData("Path", "Complete");
//        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
