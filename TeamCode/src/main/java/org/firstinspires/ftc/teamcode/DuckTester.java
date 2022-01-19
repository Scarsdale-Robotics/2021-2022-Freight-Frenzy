package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Autonomous(name = "DuckTester")
public class DuckTester extends LinearOpMode {

    DuckCV duckDetector;

    MovementController mController;
    HardwareRobot robot;

    int elevatorLevel = 0;

    @Override
    public void runOpMode() {

        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, telemetry);
        robot.clawArm.setPower(1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        duckDetector = new DuckCV(cameraMonitorViewId);

        waitForStart();

        int duckPos = -1;


        int votes[] = {0, 0, 0};


        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < 5000)) {
            duckPos = duckDetector.getDuckPosition();
            telemetry.addData("pos: ", duckPos);
//            telemetry.addData("i: ", i);

            telemetry.addData("x: ", duckDetector.duckX);
            telemetry.addData("y: ", duckDetector.duckY);
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
