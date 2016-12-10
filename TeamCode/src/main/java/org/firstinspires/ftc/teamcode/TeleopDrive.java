package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p/>`
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Teleop: Drive", group = "RoboLords")
public class TeleopDrive extends RoboLordsLinearOpMode {
    double clawOffset = 0;                       // Servo mid position
    final double CLAW_SPEED = 0.02;                   // sets rate to move servo
    static final double DRIVE_SPEED_MIN = -0.5;
    static final double DRIVE_SPEED_MAX = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        log("Robot", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left = -gamepad1.left_stick_y + gamepad1.right_stick_x;
            right = -gamepad1.left_stick_y - gamepad1.right_stick_x;

            //Clip to be within the interval [min,max]
            left = Range.clip(left, DRIVE_SPEED_MIN, DRIVE_SPEED_MAX);
            right = Range.clip(right, DRIVE_SPEED_MIN, DRIVE_SPEED_MAX);

            if (gamepad1.right_bumper) {
                robot.leftDriveMotor.setPower(RoboLordsHardware.MOTOR_FULL_POWER_FORWARD);
                robot.rightDriveMotor.setPower(RoboLordsHardware.MOTOR_FULL_POWER_REVERSE);
            } else if (gamepad1.left_bumper) {
                robot.leftDriveMotor.setPower(RoboLordsHardware.MOTOR_FULL_POWER_REVERSE);
                robot.rightDriveMotor.setPower(RoboLordsHardware.MOTOR_FULL_POWER_FORWARD);
            } else {
                robot.leftDriveMotor.setPower(left);
                robot.rightDriveMotor.setPower(right);
            }

            // Send telemetry message to signify robot running;
            log("gamepad1.right_bumper", gamepad1.right_bumper);
            log("gamepad1.left_bumper", gamepad1.left_bumper);
            log("left joystick", "%.2f", left);
            log("right joystick", "%.2f", right);
            log("left drive position:", "%7d", robot.leftDriveMotor.getCurrentPosition());
            log("right drive position:", "%7d", robot.rightDriveMotor.getCurrentPosition());

            // Use gamepad left & right Bumpers to open and close the claw
//            if (gamepad1.right_bumper) {
//                clawOffset += CLAW_SPEED;
//            } else if (gamepad1.left_bumper) {
//                clawOffset -= CLAW_SPEED;
//            }

            //launch and pickup
            if (gamepad2.left_bumper) {
                pickupParticles();
            } else if (gamepad2.right_bumper) {
                launchParticles();
            } else {
                stopPickup();
                stopLaunching();
            }

            // Move both servos to new position.  Assume servos are mirror image of each other.
//            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
//            robot.tubeServo.setPosition(robot.MID_SERVO - clawOffset);
//            robot.tubeServo.setPosition(robot.TUBE_SERVO_POSITION_START);

//             Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad2.x) {
//                robot.armMotor.setPower(robot.ARM_UP_POWER);
//                robot.rightClaw.setPosition(robot.MID_SERVO + clawOffset);
//                robot.tubeServo.setPosition(robot.TUBE_SERVO_POSITION_START);
            } else if (gamepad2.y) {
//                robot.armMotor.setPower(robot.ARM_DOWN_POWER);
//                robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);
//                robot.tubeServo.setPosition(robot.TUBE_SERVO_POSITION_DEGREE_45);
            } else if (gamepad2.b) {
//                robot.armMotor.setPower(0.0);
//                robot.rightClaw.setPosition(robot.MID_SERVO);
//                robot.tubeServo.setPosition(robot.TUBE_SERVO_POSITION_DEGREE_90);
            } else if (gamepad2.a) {
//                robot.armMotor.setPower(0.0);
//                robot.rightClaw.setPosition(robot.MID_SERVO);
//                robot.tubeServo.setPosition(robot.TUBE_SERVO_POSITION_DEGREE_180);
            }

            if (gamepad2.left_trigger > 0) {
                dropParticleBasket();
            } else if (gamepad2.right_trigger > 0) {
                liftParticleBasket();
            } else {
                stopBasket();
            }

            // Send telemetry message to signify robot running;
            log("left motor position:", "%7d", robot.leftDriveMotor.getCurrentPosition());
            log("right motor position:", "%7d", robot.rightDriveMotor.getCurrentPosition());
            log("left", "%.2f", left);
            log("right", "%.2f", right);

            log("claw", "Offset = %.2f", clawOffset);
//            log("rightClaw position:", "%.2f", robot.rightClaw.getPosition());
//            log("leftClaw position:", "%.2f", robot.leftClaw.getPosition());
//            log("tubeServo position:", "%.2f", robot.tubeServo.getPosition());
//
            log("Raw", robot.opticalDistanceSensor.getRawLightDetected());
            log("Normal", robot.opticalDistanceSensor.getLightDetected());

            if (robot.irSeekerSensor.signalDetected()) {
                log("Angle", robot.irSeekerSensor.getAngle());
                log("Strength", robot.irSeekerSensor.getStrength());
            } else {
                log("Seeker", "Signal Lost");
            }

            if (robot.touchSensor.isPressed()) {
                log("Touch", "Is Pressed");
            } else {
                log("Touch", "Is Not Pressed");
            }

            telemetry.update();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//            robot.waitForTick(40);
            idle();
        }
    }
}
