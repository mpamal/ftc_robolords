package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by RoboLords
 */
@TeleOp(name = "Auto: Drive To Press Beacon", group = "RoboLords")
public class AutoDriveToPressBeacon extends LinearOpMode {
    RoboLordsHardware robot = new RoboLordsHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        double leftPower;
        double rightPower;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Robot", "Hello Auto Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            double rawLight = robot.opticalDistanceSensor.getRawLightDetected();

            telemetry.addData("Raw", rawLight);
            telemetry.addData("Normal", robot.opticalDistanceSensor.getLightDetected());

//            Stop the robot because it is closer to the wall
            if (rawLight >= 0.6) {
                leftPower = 0;
                rightPower = 0;
            } else {
                leftPower = 1;
                rightPower = 1;
            }
            // Send telemetry message to signify robot running;
            robot.leftMotor.setPower(leftPower);
            robot.rightMotor.setPower(rightPower);

            telemetry.addData("left", "%.2f", leftPower);
            telemetry.addData("right", "%.2f", rightPower);


            if (robot.irSeekerSensor.signalDetected()) {
                telemetry.addData("Angle", robot.irSeekerSensor.getAngle());
                telemetry.addData("Strength", robot.irSeekerSensor.getStrength());
            } else {
                telemetry.addData("Seeker", "Signal Lost");
            }

            if (robot.touchSensor.isPressed()) {
                telemetry.addData("Touch", "Is Pressed");
            } else {
                telemetry.addData("Touch", "Is Not Pressed");
            }

            telemetry.update();
            idle();
        }
    }
}
