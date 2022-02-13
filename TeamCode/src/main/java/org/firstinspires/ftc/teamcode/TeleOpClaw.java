package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.vision.AutoAlignCV;

@TeleOp(name = "TeleOpClaw")
public class TeleOpClaw extends OpMode {
    MovementController mController;
    HardwareRobot robot;
    InDepSystem inDep;
    AutoAlignCV detector;

    long carouselTimer;
    boolean previousX = false;

    float driveModifier = 1;
    float rotModifier = 1;
    @Override
    public void init() {
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, this);
        inDep = new InDepSystem(robot, this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        detector = new AutoAlignCV(cameraMonitorViewId);
    }

    @Override
    public void loop() {
        telemetry.addData("frontDist: ", robot.getDistance(robot.frontDist));
        telemetry.addData("highFrontDist: ", robot.getDistance(robot.highFrontDist));
        telemetry.addData("backDist: ", robot.getDistance(robot.backDist));

        telemetry.update();
        double xMovement = 0;
        double yMovement = gamepad1.left_stick_y;

        double xLook = gamepad1.right_stick_x / 1.25;

        mController.joystickMovement(xMovement * driveModifier, -yMovement * driveModifier);
        mController.rotationalModifier(xLook * rotModifier);

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
            mController.drive(-0.025);

            if (!previousX) {
                previousX = true;
                carouselTimer = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - carouselTimer < 1000.0) {
                robot.duckSpinLeft.setPower(-0.7);
                robot.duckSpinRight.setPower(0.7);
            }
            else if (System.currentTimeMillis() - carouselTimer < 2000) {
                robot.duckSpinLeft.setPower(-1.0);
                robot.duckSpinRight.setPower(1.0);
            } else {
                robot.duckSpinRight.setPower(0);
                robot.duckSpinLeft.setPower(0);

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

        if (gamepad1.b) {
            driveModifier = 0.3f;
            rotModifier = 0.3f;
        } else if (gamepad1.a) {
            driveModifier = 1f;
            rotModifier = 1f;

        }

        if(gamepad1.y) {
            int x = detector.getXPosition();
            int width = detector.getItemWidth();
            telemetry.addData("Position: ", x);
            telemetry.addData("Width: ", width);

            if (x < 140) {
                mController.rotateInPlace(0.7);
            } else if (x > 180) {
                mController.rotateInPlace(-0.7);
            } else {
                mController.stop();
            }

            mController.update();

            telemetry.update();
        }


        mController.update();
    }
}
