package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by RoboLords
 */
@Autonomous(name = "Blue:03:Push Capball and Park at Center Vortex", group = "RoboLords")
public class AutoBlue03PushCapball extends RoboLordsLinearOpMode {
    private static final double DRIVE_SLOW_SPEED = 0.25;
    private static final double DRIVE_NORMAL_SPEED = 0.5;
    private static final double DRIVE_HIGH_SPEED = 0.75;
    private static final double TURN_SPEED = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        double timeoutSeconds;
        robot.init(hardwareMap);
        enableOpticalDistanceSensor(true);
        encoderDrive(DRIVE_SLOW_SPEED, 5);
        encoderDriveTurn45Right(DRIVE_NORMAL_SPEED);
        encoderDrive(DRIVE_NORMAL_SPEED, 45);
//        encoderDriveTurnRight(TURN_SPEED);

        idle();
    }
}
