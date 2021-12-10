package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ServoDebugger")
public class DuckAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);
        waitForStart();
        while(opModeIsActive()) {
            robot.clawRight.setPosition(-0.25);
            robot.clawLeft.setPosition(0.25);
            sleep(1000);
            robot.clawRight.setPosition(-1);
            robot.clawLeft.setPosition(1);
            sleep(1000);
        }

    }
}