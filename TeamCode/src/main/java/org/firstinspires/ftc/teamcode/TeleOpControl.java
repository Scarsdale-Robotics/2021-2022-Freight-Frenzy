package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOpReeeeee")
public class TeleOpControl extends OpMode {

    DuckCV duckDetector;

    MovementController mController;
    HardwareRobot robot;

    int elevatorLevel = 0;

    int elevatorHeight = 0;
    @Override
    public void init() {
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        duckDetector = new DuckCV(cameraMonitorViewId);
    }

    @Override
    public void loop() {

//        double xMovement = gamepad1.left_stick_x;
//        double yMovement = gamepad1.left_stick_y;
//
//        double xLook = gamepad1.right_stick_x;
//
//
//        mController.joystickMovement(xMovement, yMovement);
//
//        mController.rotationalModifier(xLook);
//        mController.update();
//
//
//        int duckPos = duckDetector.getDuckPosition();
//        telemetry.addData("Duck Pos: ", duckPos);
//
//
//        if (gamepad1.right_bumper) {
//            robot.duckSpin.setPower(0.5);
//        } else {
//            robot.duckSpin.setPower(0);
//        }
//
//
//        // raise lower elevator linear slide
////        if (gamepad1.dpad_up) {
////            robot.elevatorCable.setTargetPosition(robot.elevatorCable.getCurrentPosition() + 50);
////            robot.elevatorCable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            robot.elevatorCable.setPower(-1);
////        } else if (gamepad1.dpad_down) {
////            robot.elevatorCable.setTargetPosition(robot.elevatorCable.getCurrentPosition() - 50);
////            robot.elevatorCable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            robot.elevatorCable.setPower(1);
////        }else{
////            robot.elevatorCable.setPower(0);
////        }
//
//
//        if (gamepad1.dpad_right) {
//            if (elevatorLevel < mController.levelArray.length) elevatorLevel++;
//            mController.lift(elevatorLevel);
//        } else if (gamepad1.dpad_left) {
//            if (elevatorLevel > 0) elevatorLevel--;
//            mController.lift(elevatorLevel);
//        }
//
//        if (gamepad1.y) {
//            robot.elevatorDoor.setPosition(0);
//        }
//        if (gamepad1.b) {
//            robot.elevatorDoor.setPosition(0.5);
//        }
//        if (gamepad1.a) {
//            robot.elevatorDoor.setPosition(1);
//        }
//
//        // elevator intake
//        if (gamepad1.x) {
//            robot.elevatorIntake.setPower(-0.75);
//        } else {
//            robot.elevatorIntake.setPower(0);
//        }
//
//
//        telemetry.addData("Distance Front: ", robot.frontDist.getDistance(DistanceUnit.CM));
//        telemetry.addData("Distance Back: ", robot.backDist.getDistance(DistanceUnit.CM));
//        telemetry.addData("Distance Left: ", robot.leftDist.getDistance(DistanceUnit.CM));
//        telemetry.addData("Distance Right: ", robot.rightDist.getDistance(DistanceUnit.CM));
//        telemetry.addData("LS Y:", yMovement);
//        telemetry.addData("LS S:", xMovement);
    }
}
