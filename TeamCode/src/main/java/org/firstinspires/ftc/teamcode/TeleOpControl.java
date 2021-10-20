package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        double yMovement = -gamepad1.left_stick_y;

        double xLook = gamepad1.right_stick_x;




        mController.joystickMovement(xMovement, yMovement);

        mController.rotationalModifier(xLook);
        mController.update();



        if(gamepad1.a){
            robot.rightArm.setPosition(0);
            robot.leftArm.setPosition(0);
        }

        if(gamepad1.b){
            robot.rightArm.setPosition(1);
            robot.leftArm.setPosition(1);
        }
        telemetry.addData("Distance Front: ", robot.frontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Back: ", robot.backDist.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Left: ", robot.leftDist.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Right: ", robot.rightDist.getDistance(DistanceUnit.CM));
        telemetry.addData("LS Y:", yMovement);
        telemetry.addData("LS S:", xMovement);

    }
}