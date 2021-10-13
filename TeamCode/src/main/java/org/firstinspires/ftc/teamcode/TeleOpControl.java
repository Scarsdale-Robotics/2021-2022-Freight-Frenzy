package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class TeleOpControl extends OpMode {

    HardwareRobot robot = new HardwareRobot(hardwareMap);
    MovementController mController = new MovementController(robot, telemetry);
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y > 0.5){
            mController.moveForward(0.2);
        }
        if(gamepad1.left_stick_y < -0.5){
            mController.moveBackwards(0.2);
        }
    }
}