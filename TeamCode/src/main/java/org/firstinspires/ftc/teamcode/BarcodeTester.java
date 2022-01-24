package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DuckTester")
public class BarcodeTester extends LinearOpMode {

    BarcodeCV duckDetector;

    MovementController mController;
    HardwareRobot robot;

    int elevatorLevel = 0;

    @Override
    public void runOpMode() {

        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, this);
        robot.clawArm.setPower(1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        duckDetector = new BarcodeCV(cameraMonitorViewId);

        waitForStart();

        int duckPos = -1;


        int[] votes = {0, 0, 0};

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < 5000)) {
            duckPos = duckDetector.getBarcodePosition();

            telemetry.addData("pos: ", duckPos);
            telemetry.addData("x: ", duckDetector.itemX);
            telemetry.addData("y: ", duckDetector.itemY);
            telemetry.update();

            if (duckPos != -1) {
                votes[duckPos]++;
            }
        }
        int max = -1;
        duckPos = -1;
        for (int i = 0; i < 3; i++) {
            if (votes[i] >= max) {
                max = (votes[i]);
                duckPos = i;
            }
        }
        telemetry.addData("0: ", votes[0]);
        telemetry.addData("1: ", votes[1]);
        telemetry.addData("2: ", votes[2]);

        telemetry.addData("final: ", duckPos);
        telemetry.update();

        while (opModeIsActive()) ;
    }
}
