package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.AutoAlignCV;
import org.firstinspires.ftc.teamcode.vision.BarcodeCV;

@Autonomous(name = "AlignTester")
public class AlignTester extends LinearOpMode {
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        AutoAlignCV detector = new AutoAlignCV(cameraMonitorViewId);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Position: ", detector.getXPosition());
        }
    }
}
