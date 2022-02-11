package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpClaw")
public class TeleOpClaw extends OpMode {
    MovementController mController;
    HardwareRobot robot;
    InDepSystem inDep;

    long carouselTimer;
    boolean previousX = false;

    @Override
    public void init() {
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, this);
        inDep = new InDepSystem(robot, this);
    }

    @Override
    public void loop() {
        telemetry.addData("frontDist: ", robot.getDistance(robot.frontDist));
        telemetry.update();
        double xMovement = 0;
        double yMovement = gamepad1.left_stick_y;

        double xLook = gamepad1.right_stick_x / 1.25;

        mController.joystickMovement(xMovement, -yMovement);
        mController.rotationalModifier(xLook);

        if (gamepad2.left_trigger > 0.01) {
            robot.clawArm.setTargetPosition(robot.clawArm.getCurrentPosition() - (int) (100 * gamepad2.left_trigger));
        }
        if (gamepad2.right_trigger > 0.01) {
            robot.clawArm.setTargetPosition(robot.clawArm.getCurrentPosition() + (int) (100 * gamepad2.right_trigger));
        }
        if (gamepad2.dpad_left) {
            robot.clawArm.setTargetPosition(3600);
        } else if (gamepad2.dpad_up) {
            robot.clawArm.setTargetPosition(1625);
        } else if (gamepad2.dpad_right) {
            robot.clawArm.setTargetPosition(300);
        } else if (gamepad2.dpad_down) {
            robot.clawArm.setTargetPosition(0);
        }

        //servos
        if (gamepad2.y) {
            inDep.openClaw();
        } else if (gamepad2.b) {
            inDep.closeClaw();
        }

        if (gamepad1.x) {
            mController.drive(-0.1);

            if (!previousX) {
                previousX = true;
                carouselTimer = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - carouselTimer < 850.0) {
                robot.duckSpinLeft.setPower(-0.5);
                robot.duckSpinRight.setPower(0.5);
            }
            if (System.currentTimeMillis() - carouselTimer < 2000) {
                robot.duckSpinLeft.setPower(-1.0);
                robot.duckSpinRight.setPower(1.0);
            }else{
                robot.duckSpinRight.setPower(0);
                robot.duckSpinRight.setPower(0);

            }
        } else {
            robot.duckSpinLeft.setPower(0);
            previousX = false;
            robot.duckSpinRight.setPower(0);
        }

        // reset encoders in case of emergency
        if (gamepad2.x && gamepad2.a) {
            robot.clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.clawArm.setTargetPosition(0);
            robot.clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gamepad2.rumble(2000);
            gamepad1.rumble(2000);
        }

        mController.update();
    }
}
