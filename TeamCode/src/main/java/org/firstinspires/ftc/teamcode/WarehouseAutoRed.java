package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.AutoAlignCV;
import org.firstinspires.ftc.teamcode.vision.BarcodeCV;

@Autonomous(name = "WarehouseRed")
public class WarehouseAutoRed extends LinearOpMode {

    BarcodeCV barcodeDetector;
    AutoAlignCV detector;

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

        inDep.setClawPosition(0.2, 0.8);

        waitForStart();


        //grip the cube

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        barcodeDetector = new BarcodeCV(cameraMonitorViewId);

        // detect team shipping element
        int bestPos = barcodeDetector.getBarcodePosition();

        //move back from wall towards barcode
        mController.driveByEncoders(-1, 400);
//        mController.driveByDistance(-3.9, robot.frontDist, 5, false);
        barcodeDetector.close();
        detector = new AutoAlignCV(cameraMonitorViewId);
        // Set claw arm to correct position by duckLevel
        inDep.liftToHubLevel(bestPos);

        //rotate towards shipping hub
        mController.rotateToByIMU(35);
        inDep.waitForArm();

        //drive to alliance shipping hub
        mController.driveByEncoders(-1, 1100);

        //open claw dropping the cube. Delay because of servo latency
        inDep.setClawPosition(0, 1);


        //drive away from alliance shipping hub
        mController.rotateToByIMU(0);
        mController.driveByEncoders(1, 700);


        //rotate to face the warehouse and lower arm
        inDep.setArmPosition(2500);
        mController.rotateToByIMU(90);

        // Drive into the warehouse
//        mController.drive(1);
//        mController.update();
//        while (mController.opModeIsActive() && (robot.getDistance(robot.highFrontDist) > 24 || robot.getDistance(robot.highFrontDist) < 5)){
//            telemetry.addData("Dist: ", robot.getDistance(robot.highFrontDist));
//            telemetry.update();
//        }
//        mController.stop();
        mController.driveByTime(1, 2100);


        autoPickup();

        //move back
        mController.driveByEncoders(-1, 800);
        mController.rotateToByIMU(90);

        inDep.setArmPosition(1000);

        // Drive Back over barrrier
        mController.driveByTime(-1, 2200);


        // Drive
        inDep.setArmPosition(inDep.levels[2]);
        mController.rotateToByIMU(0);
        autoAlign();

        mController.driveByDistance(-0.4, robot.highFrontDist, 18, false);
        //Open claw
        inDep.waitForArm();
        inDep.openClaw();
        sleep(200);


        mController.rotateToByIMU(80);

        inDep.setArmPosition(3000);

        mController.driveByTime(1, 2200);
        while (opModeIsActive()) ;
    }

    private void autoPickup() {
        double widthSensorToClaw = 3.317;
        inDep.setClawPosition(0.75, 0.25);

        mController.rotateToByIMU(67);

        long bailTimer = System.currentTimeMillis();
        while (robot.getDistance(robot.frontDist) > 6 && opModeIsActive() && System.currentTimeMillis() - bailTimer < 2000) {
            mController.drive(0.5);
            long netTime = System.currentTimeMillis() - bailTimer;
            double rotPower = 0.15;
            mController.rotationalModifier(rotPower);
            mController.update();

            telemetry.addData("FrontDist: ", robot.getDistance(robot.frontDist));
            telemetry.update();
        }

        float dist = (float) robot.getDistance(robot.frontDist);
        mController.stop();


        float angleOffset =  (float) Math.toDegrees(Math.atan(widthSensorToClaw / dist));
        angleOffset = 10;

        telemetry.addData("Dist", dist);
        telemetry.addData("Angle: ", robot.getImuAngle());
        telemetry.addData("Offset: ", angleOffset);
        mController.rotateToByIMU(robot.getImuAngle() - angleOffset);
        telemetry.addData("After: ", robot.getImuAngle());
        telemetry.update();

        mController.driveByEncoders(-0.5, -600);

        inDep.setArmPosition(150);
        while (robot.clawArm.isBusy() && opModeIsActive()) ;


        mController.driveByEncoders(0.5, 875);

        inDep.closeClaw();
        sleep(200);

    }


    private void autoAlign() {
        long start = System.currentTimeMillis();

        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            int x = detector.getXPosition();
            int width = detector.getItemWidth();
            telemetry.addData("Position: ", x);
            telemetry.addData("Width: ", width);

            if (x < 140) {
                mController.rotateInPlace(0.7);
            } else if (x > 180) {
                mController.rotateInPlace(-0.7);
            } else {
                return;
            }

            mController.update();
            telemetry.update();
        }
    }
}
