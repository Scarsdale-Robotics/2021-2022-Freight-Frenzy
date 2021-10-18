package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AlongWall")
public class AlongWall extends LinearOpMode {
    public void runOpMode() {
        waitForStart();
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);
        while (opModeIsActive()) {

            while (robot.frontDist.getDistance(DistanceUnit.CM) >= 25) {
                mController.drive(0.2);
                mController.update();
            }
            while (robot.rightDist.getDistance(DistanceUnit.CM) >= 25) {
                mController.strafe(0.2);
                mController.update();
            }
            while (robot.backDist.getDistance(DistanceUnit.CM) >= 25) {
                mController.drive(-0.2);
                mController.update();
            }
            while (robot.leftDist.getDistance(DistanceUnit.CM) >= 25) {
                mController.strafe(-0.2);
                mController.update();
            }
        }
    }
}

