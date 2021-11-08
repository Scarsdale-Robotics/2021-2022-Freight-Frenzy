package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOpReeeeee")
public class TeleOpControl extends OpMode {

    DuckCV duckDetector = new DuckCV();

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

        mController.rotationalModifier(xLook);
        mController.update();


        if (gamepad1.a) {

            robot.rightClaw.setPosition(0.5);
            robot.leftClaw.setPosition(0);
        }

        if (gamepad1.b) {
            robot.rightClaw.setPosition(0);
            robot.leftClaw.setPosition(0.5);
        }

        if (gamepad1.right_trigger > 0.1) {
            robot.clawArm.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            robot.clawArm.setPower(-gamepad1.left_trigger);
        } else {
            robot.clawArm.setPower(0);
        }


//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        duckDetector.beep(cameraMonitorViewId);
//        int duckPos = duckDetector.getDuckPosition();
//        telemetry.addData("Duck Pos: ", duckPos);


        telemetry.addData("Distance Front: ", robot.frontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Back: ", robot.backDist.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Left: ", robot.leftDist.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Right: ", robot.rightDist.getDistance(DistanceUnit.CM));
        telemetry.addData("LS Y:", yMovement);
        telemetry.addData("LS S:", xMovement);

        if(gamepad1.right_bumper){
            robot.spinThing.setPower(0.5);
        }else{
            robot.spinThing.setPower(0);
        }

    }
}