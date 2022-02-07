package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.BarcodeCV;

@Autonomous(name = "WarehouseRed")
public class WarehouseAutoRed extends LinearOpMode {

    BarcodeCV barcodeDetector;

    MovementController mController;
    HardwareRobot robot;
    InDepSystem inDep;

    long startTimer;

    @Override
    public void runOpMode() {

        // Init
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, this);
        inDep = new InDepSystem(robot, this);

        waitForStart();
        //grip the cube
        inDep.setClawPosition(0.2, 0.80);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        barcodeDetector = new BarcodeCV(cameraMonitorViewId);

        // detect team shipping element
        int bestPos = barcodeDetector.getBarcodePosition();

        //move back from wall towards barcode
        mController.driveByDistance(-3.9, robot.frontDist, 5, false);

        // Set claw arm to correct position by duckLevel
        inDep.liftToHubLevel(bestPos);

        //rotate towards shipping hub
        mController.rotateToByIMU(0.2, 32);
        inDep.waitForArm();

        //drive to alliance shipping hub
        mController.driveByEncoders(-1, 2300);

        //open claw dropping the cube. Delay because of servo latency
        inDep.setClawPosition(0, 1);
        inDep.waitForClaw();


        inDep.setClawPosition(0, 1);
//        inDep.waitForClaw();


        //drive away from alliance shipping hub
        mController.driveByEncoders(1, 1000);


        //rotate to face the warehouse and lower arm
        inDep.setArmPosition(1000);
        mController.rotateToByIMU(0.2, 90);
        inDep.openClaw();

        // Drive backwards because there is not enough room accelerate to full speed to get over barriers
        mController.driveByTime(-0.6, 500);
        mController.driveByEncoders(1, 300);
        sleep(200);

        // Drive into the warehouse
        mController.driveByTime(1, 2000);
        mController.rotateToByIMU(0.2, 90);


        autoPickup();

        //move back
        mController.driveByEncoders(-1, 500);
        mController.rotateToByIMU(0.2, 90);
        mController.driveByEncoders(-1, 500);

        inDep.setArmPosition(1000);

        while(opModeIsActive());

    }

    private void autoPickup() {
        double widthSensorToClaw = 3.317;

        mController.rotateToByIMU(0.2, 90);

        mController.driveByDistance(0.5, robot.frontDist, 20, true);
        mController.rotateToByIMU(0.2, 75);

        inDep.setArmPosition(1000);
        inDep.openClaw();
        long bailTimer = System.currentTimeMillis();
        while (robot.getDistance(robot.frontDist) > 6 && opModeIsActive() && System.currentTimeMillis() - bailTimer < 2000) {
            mController.drive(0.3);
            long netTime = System.currentTimeMillis() - bailTimer;
            double rotPower = 0.1;
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
        mController.rotateToByIMU(0.2, robot.getImuAngle() + angleOffset);
        telemetry.addData("After: ", robot.getImuAngle());
        telemetry.update();

        mController.driveByEncoders(-0.5, -500);

        inDep.openClaw();
        inDep.setArmPosition(20);
        while(robot.clawArm.isBusy() && opModeIsActive());

        mController.driveByEncoders(0.5, (int) (56 * dist) + 800);


        //spin and pick up

        mController.drive(0.3);
        mController.rotationalModifier(1);
        mController.update();
        mController.sleep(400);
        inDep.closeClaw();
        mController.stop();
        mController.sleep(1000);
    }
}
