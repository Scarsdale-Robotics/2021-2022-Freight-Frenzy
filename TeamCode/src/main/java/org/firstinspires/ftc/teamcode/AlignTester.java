package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.vision.AutoAlignCV;

@Autonomous(name = "AlignTester")
public class AlignTester extends LinearOpMode {
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        AutoAlignCV detector = new AutoAlignCV(cameraMonitorViewId);

        while (opModeIsActive()) {
            int x = detector.getXPosition();
            telemetry.addData("Position: ", x);


            if (x < 140) {
                mController.pivotOnRight(-0.2);
                mController.update();
            } else if (x > 180) {
                mController.pivotOnLeft(-0.2);
                mController.update();
            } else {
                mController.stop();
                mController.update();
                break;
            }

        }
    }
}
