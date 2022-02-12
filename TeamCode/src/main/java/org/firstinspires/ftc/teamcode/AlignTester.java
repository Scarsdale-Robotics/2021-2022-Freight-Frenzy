package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.AutoAlignCV;

@Autonomous(name = "AlignTester")
public class AlignTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        AutoAlignCV detector = new AutoAlignCV(cameraMonitorViewId, true);

        waitForStart();

        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, this);

        while (opModeIsActive()) {
            int x = detector.getXPosition();
            int width = detector.getItemWidth();
            telemetry.addData("Position: ", x);
            telemetry.addData("Width: ", width);

            if (x < 140) {
                mController.pivotOnLeft(-0.7);
            } else if (x > 180) {
                mController.pivotOnRight(-0.7);
            } else {
                if(width > 30) {
                    mController.drive(-0.7);
                } else {
                    mController.stop();
                }
            }

            mController.update();

            telemetry.update();
        }
    }
}
