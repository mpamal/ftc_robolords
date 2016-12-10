package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Red:03:Push Capball and Park at Center Vortex", group = "RoboLords")
public class AutoRed03PushCapball extends RoboLordsLinearOpMode {
    private static final double DRIVE_SLOW_SPEED = 0.25;
    private static final double DRIVE_NORMAL_SPEED = 0.5;
    private static final double DRIVE_HIGH_SPEED = 0.75;
    private static final double TURN_SPEED = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        double timeoutSeconds;
        robot.init(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            enableOpticalDistanceSensor(true);
            encoderDrive(DRIVE_SLOW_SPEED, 23);
            encoderDriveTurn45Left(DRIVE_NORMAL_SPEED);
            encoderDrive(DRIVE_NORMAL_SPEED, 34);
//        encoderDriveTurnRight(TURN_SPEED);
        }
        idle();
    }
}
