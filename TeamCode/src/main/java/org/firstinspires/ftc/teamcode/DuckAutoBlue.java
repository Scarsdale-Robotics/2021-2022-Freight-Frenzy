package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.BarcodeCV;

@Autonomous(name = "DuckAutoBlue")
public class DuckAutoBlue extends LinearOpMode {

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

        // detect team shipping element
        int bestPos = barcodeDetector.getBarcodePosition();

        //move back from wall towards barcode
        mController.driveByDistance(-0.5, robot.frontDist, 10, false);

        // Set claw arm to correct position by duckLevel
        inDep.liftToHubLevel(bestPos);

        //turn to shipping hub
        mController.rotateToByIMU(43);
        inDep.waitForArm();

        //drive to alliance shipping hub
        mController.driveByEncoders(-0.7, 1600);

        //open claw dropping the cube. Delay because of servo latency
        inDep.setClawPosition(0, 1);
        inDep.waitForClaw();

        //Set claw position back
        inDep.setClawPosition(0, 1);


        //drive away from alliance shipping hub
        mController.driveByTime(0.7, 900);

        //rotate towards the carousel wall
        inDep.liftToBarrier();
        mController.rotateToByIMU( -90);
        inDep.openClaw();

        // Back up to the carousel wall
        mController.driveByDistance(-0.6, robot.backDist, 12, true);

        // Rotate so back faces the carousel
        mController.rotateToByIMU( -180);

        //Drive to the carousel
        mController.driveByDistance(-0.6, robot.backDist, 14.5, true);

        // SPIN THE DUCK
        robot.duckSpinLeft.setPower(0.3);
        robot.duckSpinRight.setPower(-0.3);
        sleep(5000);


        //Drive forward away from the wall a bit
        mController.driveByDistance(0.4, robot.backDist, 20, false);

        //Park
        mController.driveByDistance(0.4, robot.backDist, 25.5, false);
    }
}
