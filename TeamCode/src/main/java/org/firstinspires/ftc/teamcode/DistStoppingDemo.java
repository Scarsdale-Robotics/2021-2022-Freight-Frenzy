package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "DistStoppingDemo_Auto")
public class DistStoppingDemo extends LinearOpMode {

    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);

        robot.imu.getAngularOrientation();

        MovementController mController = new MovementController(robot, telemetry);
        while (opModeIsActive()) {
            if (robot.frontDist.getDistance(DistanceUnit.CM) > 50) {
                mController.drive(1);
            } else {
                mController.stop();


                stop();
            }
            mController.update();
        }
    }
}