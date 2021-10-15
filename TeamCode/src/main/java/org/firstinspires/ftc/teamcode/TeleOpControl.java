package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class TeleOpControl extends OpMode {

    MovementController mController;
    HardwareRobot robot;

    @Override
    public void init() {
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, telemetry);
    }

    @Override
    public void loop() {
        double xMovement = gamepad1.left_stick_x;
        double yMovement = gamepad1.left_stick_y;

        double xLook = gamepad1.right_stick_x;

        mController.joystickMovement(xMovement, yMovement);

        mController.rotate(xLook);

        mController.update();


    }
}