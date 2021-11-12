package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="WarehouseRed")
public class WarehouseAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);

        while (opModeIsActive() && robot.rightDist.getDistance(DistanceUnit.INCH) > 24) {
            mController.strafe(1);
            mController.update();
        }
        mController.stop();
        mController.update();

    }
}
