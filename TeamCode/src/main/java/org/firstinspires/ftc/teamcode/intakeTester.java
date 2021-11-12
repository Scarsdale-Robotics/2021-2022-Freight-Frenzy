package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import duck
@TeleOp(name = "intakeTester")
public class intakeTester extends OpMode {

    MovementController mController;
    HardwareRobot robot;

    @Override
    public void init() {
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) robot.elevatorCable.setPower(1);
        else if (gamepad1.left_bumper) robot.elevatorCable.setPower(-1);
        else robot.elevatorCable.setPower(0);

        if (gamepad1.a) {
            robot.elevatorDoor.setPosition(-1);
            telemetry.addData("Door: ", "Up");
        }
        if (gamepad1.b) {
            robot.elevatorDoor.setPosition(1);
            telemetry.addData("Door: ", "Down");

        }
        telemetry.update();
    }


}