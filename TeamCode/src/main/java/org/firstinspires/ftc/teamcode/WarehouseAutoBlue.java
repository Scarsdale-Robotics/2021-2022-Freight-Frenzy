package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="WarehouseBlue")
public class WarehouseAutoBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);

        mController.strafe(-1.0);
        mController.update();

        while (robot.leftDist.getDistance(DistanceUnit.CM) > 68.58);

        mController.stop();
        mController.update();
    }
}
