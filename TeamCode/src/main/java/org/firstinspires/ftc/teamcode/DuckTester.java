package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DuckTester")
public class DuckTester extends OpMode {

    DuckCV duckDetector;

    MovementController mController;
    HardwareRobot robot;

    int elevatorLevel = 0;

    @Override
    public void init() {
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, telemetry);
        robot.clawArm.setPower(1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        duckDetector = new DuckCV(cameraMonitorViewId);


    }

    @Override
    public void loop() {
        telemetry.addData("Pos: ", duckDetector.getDuckPosition());
        telemetry.update();
    }
}
