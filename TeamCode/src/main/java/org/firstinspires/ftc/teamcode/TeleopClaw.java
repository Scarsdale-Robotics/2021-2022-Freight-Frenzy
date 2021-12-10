package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleopClaw")
public class TeleopClaw extends OpMode {

    DuckCV duckDetector;

    MovementController mController;
    HardwareRobot robot;

    int elevatorLevel = 0;

    @Override
    public void init() {
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        duckDetector = new DuckCV(cameraMonitorViewId);

        robot.elevatorCable.setTargetPosition(0);

    }

    @Override
    public void loop() {

        double xMovement = gamepad1.left_stick_x;
        double yMovement = gamepad1.left_stick_y;

        double xLook = gamepad1.right_stick_x/2;


        mController.joystickMovement(xMovement, yMovement);

        mController.rotationalModifier(xLook);
        mController.update();


        int duckPos = duckDetector.getDuckPosition();
        telemetry.addData("Duck Pos: ", duckPos);


        if (gamepad1.dpad_up) {
            robot.clawArm.setTargetPosition(robot.clawArm.getCurrentPosition() - 100);
        } else if (gamepad1.dpad_down) {
            robot.clawArm.setTargetPosition(robot.clawArm.getCurrentPosition() + 100);
        }else{
            robot.clawArm.setPower(1);
        }

        //servos

        if (gamepad1.y) { //open
            robot.clawLeft.setPosition(0.6f);
            robot.clawRight.setPosition(0.5f);
        }
        if (gamepad1.b) { //close
            robot.clawLeft.setPosition(1.0f);
            robot.clawRight.setPosition(0f);
        }



        if (gamepad1.x) {
            robot.duckSpinLeft.setPower(-0.5);
            robot.duckSpinRight.setPower(0.5);

        } else {
            robot.duckSpinLeft.setPower(0);
            robot.duckSpinRight.setPower(0);
        }

        telemetry.addData("Arm Endcoder: ", robot.clawArm.getCurrentPosition());
        telemetry.update();
    }
}
