//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@TeleOp(name = "ClawTester")
//public class ClawTester extends OpMode {
//
//    DuckCV duckDetector;
//
//    MovementController mController;
//    HardwareRobot robot;
//
//    int elevatorLevel = 0;
//
//    @Override
//    public void init() {
//        robot = new HardwareRobot(hardwareMap);
//        mController = new MovementController(robot, telemetry);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        duckDetector = new DuckCV(cameraMonitorViewId);
//
//
//        robot.clawArm.setTargetPosition(0);
//    }
//
//    @Override
//    public void loop() {
//        // raise lower elevator linear slide
//        if (gamepad1.dpad_up) {
//            robot.clawArm.setTargetPosition(robot.clawArm.getCurrentPosition() - 100);
//        } else if (gamepad1.dpad_down) {
//            robot.clawArm.setTargetPosition(robot.clawArm.getCurrentPosition() + 100);
//        }else{
//            robot.clawArm.setPower(1);
//        }
//
//        //servos
//
//        if (gamepad1.y) { //open
//            robot.clawLeft.setPosition(1);
//            robot.clawRight.setPosition(0);
//        }
//        if (gamepad1.b) { //close
//            robot.clawLeft.setPosition(0);
//            robot.clawRight.setPosition(1);
//        }
//
//        if (gamepad1.x) {
//            robot.duckSpinLeft.setPower(0.5);
//            robot.duckSpinRight.setPower(0.5);
//
//        } else {
//            robot.duckSpinLeft.setPower(0);
//            robot.duckSpinRight.setPower(0);
//
//        }
//
//        telemetry.addData("Arm Endcoder: ", robot.clawArm.getCurrentPosition());
//        telemetry.update();
//    }
//}
