package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "beep")
public class Color_Sensor_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);

        mController.drive(1);
        mController.update();

        while ((robot.color.red() <= 30 || robot.color.red() >= 60) && (robot.color.blue() <= 30 || robot.color.blue() >= 60) && (robot.color.green() <= 30 || robot.color.green() >= 60)) {

        }
        mController.stop();
        mController.update();
    }
}
