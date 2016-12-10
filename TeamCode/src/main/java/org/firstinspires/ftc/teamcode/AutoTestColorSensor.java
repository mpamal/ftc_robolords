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

        log("Status", "Hello driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        disableAllSensors();
        enableColorSensors(true);
        enableOpticalDistanceSensor(true);
        encoderDrive(DRIVE_SLOW_SPEED, 60);
        if (isBlueLightDetected() || isWhiteLightDetected()) {
            disableAllSensors();
            encoderDrive(DRIVE_SLOW_SPEED, -12);
        }

        enableColorSensors(true);
        while (opModeIsActive()) {
            log("ODS, raw", robot.opticalDistanceSensor.getRawLightDetected());
            log("ODS, normal", robot.opticalDistanceSensor.getLightDetected());

            log("Clear", robot.leftColorSensor.alpha());
            log("Red  ", robot.leftColorSensor.red());
            log("Green", robot.leftColorSensor.green());
            log("Blue ", robot.leftColorSensor.blue());

//            log("Clear", robot.rightColorSensor.alpha());
//            log("Red  ", robot.rightColorSensor.red());
//            log("Green", robot.rightColorSensor.green());
//            log("Blue ", robot.rightColorSensor.blue());
            telemetry.update();
            idle();
        }
    }
}
