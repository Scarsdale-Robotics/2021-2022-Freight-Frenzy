package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="WarehouseRed")
public class WarehouseAutoRed extends LinearOpMode {

    DuckCV duckDetector;

    MovementController mController;
    HardwareRobot robot;

    int elevatorLevel = 0;
    long startTimer;

    @Override
    public void runOpMode() {

        //Init
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, telemetry);
        robot.clawArm.setPower(1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        duckDetector = new DuckCV(cameraMonitorViewId);

        waitForStart();
        int duckPos = duckDetector.getDuckPosition();




        while(opModeIsActive());




    }
}
