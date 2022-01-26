package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DuckAutoRed")
public class DuckAutoRed extends LinearOpMode {

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
        inDep.setClawPosition(0.2, 0.80);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        barcodeDetector = new BarcodeCV(cameraMonitorViewId);

        // detect ducks, taking the position with the most occurrences within 1.5 seconds
        int[] votes = {0, 0, 0};
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTimer < 1500)) {
            int barcodePosition = barcodeDetector.getBarcodePosition();

            telemetry.addData("pos: ", barcodePosition);
            telemetry.addData("x: ", barcodeDetector.itemX);
            telemetry.addData("y: ", barcodeDetector.itemY);
            telemetry.update();

            if (barcodePosition != -1) {
                votes[barcodePosition]++;
            }
        }

        int bestPos = 2;
        for (int i = 0; i < votes.length; i++) {
            if (votes[i] >= votes[bestPos]) {
                bestPos = i;
            }
        }

        //move back
        mController.driveByDistance(-0.5, robot.frontDist, 5, false);
        // Set claw arm to correct position by duckLevel
        inDep.liftToHubLevel(bestPos);

        //turn to shipping hub
        mController.rotateToByIMU(0.2, -35);
        inDep.waitForArm();

        //drive to alliance shipping hub
        mController.driveByEncoders(-0.7, 3200);
        //open claw dropping the cube. Delay because of servo latency
        inDep.setClawPosition(0, 1);
        inDep.waitForClaw();


        inDep.setClawPosition(0, 1);
//        inDep.waitForClaw();


        //drive away from alliance shipping hub
        mController.driveByTime(0.7, 900);

        //rotate to face the warehouse and lower arm
        inDep.liftToBarrier();
        mController.rotateToByIMU(-0.2, 90);
        inDep.openClaw();

        // Drive backwards because there is not enough room accelerate to full speed to get over barriers
        mController.driveByDistance(-0.6, robot.backDist, 12, true);

        mController.rotateToByIMU(-0.2, 0);
        mController.driveByDistance(-0.6, robot.backDist, 15, true);
        robot.duckSpinLeft.setPower(0.3);
        robot.duckSpinRight.setPower(-0.3);
        sleep(5000);
        mController.driveByDistance(0.4, robot.backDist, 20, true);



        mController.rotateToByIMU(0.2, 0);


    }
}
