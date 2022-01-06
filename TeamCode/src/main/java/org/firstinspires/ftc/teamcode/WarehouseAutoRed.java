package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="WarehouseRed")
public class WarehouseAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);

        while (opModeIsActive() && robot.frontDist.getDistance(DistanceUnit.INCH) > 24) {
            mController.joystickMovement(0, -1);
            mController.update();
            telemetry.addData("front: ", robot.frontDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("left: ", robot.leftDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("right: ", robot.rightDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("back: ", robot.backDist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        mController.stop();
        mController.update();

        while(opModeIsActive()){
            telemetry.addData("front: ", robot.frontDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("left: ", robot.leftDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("right: ", robot.rightDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("back: ", robot.backDist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }
}
