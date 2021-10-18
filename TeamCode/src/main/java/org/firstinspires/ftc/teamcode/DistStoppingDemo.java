package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "DistStoppingDemo_Auto")
public class DistStoppingDemo extends LinearOpMode {

    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);
        while (opModeIsActive()) {
            if (robot.testDist.getDistance(DistanceUnit.CM) > 50) {
                mController.drive(1);
            } else {
                mController.stop();

                robot.testServo.setPosition(1.0);

                while(robot.testServo.getPosition()<0.9);
                robot.testServo.setPosition(0);
                stop();
            }
            mController.update();
        }
    }
}