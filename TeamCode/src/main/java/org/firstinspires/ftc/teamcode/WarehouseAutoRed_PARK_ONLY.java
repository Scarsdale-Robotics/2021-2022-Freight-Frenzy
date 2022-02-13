package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.AutoAlignCV;
import org.firstinspires.ftc.teamcode.vision.BarcodeCV;

@Autonomous(name = "WarehouseAutoRedPARKONLY")
public class WarehouseAutoRed_PARK_ONLY extends LinearOpMode {

    BarcodeCV barcodeDetector;

    MovementController mController;
    HardwareRobot robot;
    InDepSystem inDep;

    long startTimer;
    int cameraMonitorViewId;

    @Override
    public void runOpMode() {

        // Init
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, this);
        inDep = new InDepSystem(robot, this);

        waitForStart();


        //grip the cube
        inDep.setClawPosition(0.2, 0.80);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        barcodeDetector = new BarcodeCV(cameraMonitorViewId);

        // detect team shipping element
        int bestPos = barcodeDetector.getBarcodePosition();

        //move back from wall towards barcode
        mController.driveByEncoders(-1, 400);
//        mController.driveByDistance(-3.9, robot.frontDist, 5, false);

        // Set claw arm to correct position by duckLevel
        inDep.liftToHubLevel(bestPos);

        //rotate towards shipping hub
        mController.rotateToByIMU( 35);
        inDep.waitForArm();

        //drive to alliance shipping hub
        mController.driveByEncoders(-1, 1100);

        //open claw dropping the cube. Delay because of servo latency
        inDep.setClawPosition(0, 1);
        inDep.waitForClaw();


        inDep.setClawPosition(0, 1);
//        inDep.waitForClaw();


        //drive away from alliance shipping hub
        mController.driveByEncoders(1, 800);


        //rotate to face the warehouse and lower arm
        inDep.setArmPosition(3000);
        mController.rotateToByIMU( 90);
        inDep.openClaw();

        // Drive backwards because there is not enough room accelerate to full speed to get over barriers

        // Drive into the warehouse
        mController.driveByTime(1, 2000);
//        mController.driveByEncoders(1, 2200);
        mController.stop();


        while (opModeIsActive());
    }

    private void autoPickup() {
        double widthSensorToClaw = 3.317;

        inDep.setArmPosition(1000);
        inDep.openClaw();
        inDep.setClawPosition(0.7, 0.3);

        mController.rotateToByIMU( 75);

        long bailTimer = System.currentTimeMillis();
        while (robot.getDistance(robot.frontDist) > 6 && opModeIsActive() && System.currentTimeMillis() - bailTimer < 2000) {
            mController.drive(0.5);
            long netTime = System.currentTimeMillis() - bailTimer;
            double rotPower = 0.15;
            if (netTime % 3000 < 1.5) rotPower *= -1;
            mController.rotationalModifier(rotPower);
            mController.update();
        }

        float dist = (float) robot.getDistance(robot.frontDist);
        mController.stop();


        float angleOffset = 90 - (float) Math.toDegrees(Math.atan(dist / widthSensorToClaw));
        angleOffset = 0;

        telemetry.addData("Angle: ", robot.getImuAngle());
        telemetry.addData("Offset: ", angleOffset);
        mController.rotateToByIMU( robot.getImuAngle() + angleOffset);
        telemetry.addData("After: ", robot.getImuAngle());
        telemetry.update();

        mController.driveByEncoders(-0.5, -500);

        inDep.openClaw();
        inDep.setArmPosition(40);
        while (robot.clawArm.isBusy() && opModeIsActive());

        mController.driveByEncoders(0.5, (int) (56 * dist) + 500);


        //spin and pick up

//        mController.drive(0.3);
//        mController.rotationalModifier(1);
//        mController.update();
//        mController.sleep(400);
        inDep.closeClaw();
        mController.stop();
        mController.sleep(1000);
    }



    private void autoAlign(){
        AutoAlignCV detector = new AutoAlignCV(cameraMonitorViewId);
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            int x = detector.getXPosition();
            int width = detector.getItemWidth();
            telemetry.addData("Position: ", x);
            telemetry.addData("Width: ", width);

            if (x < 140) {
                mController.pivotOnLeft(-0.7);
            } else if (x > 180) {
                mController.pivotOnRight(-0.7);
            } else {
                return;
            }

            mController.update();

            telemetry.update();
        }
    }
}
