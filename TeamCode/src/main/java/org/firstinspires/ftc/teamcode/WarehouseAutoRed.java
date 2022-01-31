package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        mController.driveByDistance(-0.5, robot.frontDist, 5, false);

        // Set claw arm to correct position by duckLevel
        inDep.liftToHubLevel(bestPos);

        //rotate towards shipping hub
        mController.rotateToByIMU(0.2, 32);
        inDep.waitForArm();

        //drive to alliance shipping hub
        mController.driveByEncoders(-0.7, 2300);

        //open claw dropping the cube. Delay because of servo latency
        inDep.setClawPosition(0, 1);
        inDep.waitForClaw();


        inDep.setClawPosition(0, 1);
//        inDep.waitForClaw();


        //drive away from alliance shipping hub
        mController.driveByTime(0.7, 900);

        //rotate to face the warehouse and lower arm
        inDep.liftToBarrier();
        mController.rotateToByIMU(0.2, 90);
        inDep.openClaw();

        // Drive backwards because there is not enough room accelerate to full speed to get over barriers
        mController.driveByTime(-0.6, 500);
        sleep(200);

        // Drive into the warehouse
        mController.driveByTime(1, 2000);
        mController.rotateToByIMU(0.2, 90);

    }
}
