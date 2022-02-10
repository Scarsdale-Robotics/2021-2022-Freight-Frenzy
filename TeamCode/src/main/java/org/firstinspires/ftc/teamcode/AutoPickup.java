package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.BarcodeCV;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Collections;

@Autonomous(name = "Autopickup")
public class AutoPickup extends LinearOpMode {

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

        double widthSensorToClaw = 3.317;

        mController.rotateToByIMU( 90);

        inDep.setArmPosition(1000);
        inDep.openClaw();
        long bailTimer = System.currentTimeMillis();
        while (robot.getDistance(robot.frontDist) > 6 && opModeIsActive() && System.currentTimeMillis() - bailTimer < 2000) {
            mController.drive(0.2);
            long netTime = System.currentTimeMillis() - bailTimer;
            double rotPower = 0.2;
            if(netTime % 3000 < 1.5) rotPower *= -1;
            mController.rotationalModifier(rotPower);
            mController.update();
        }

        float dist = (float)robot.getDistance(robot.frontDist);
        mController.stop();

        inDep.openClaw();
        inDep.setArmPosition(35);
        float angleOffset = 90 - (float)Math.toDegrees(Math.atan(dist/widthSensorToClaw));
//        angleOffset = 0;
        mController.rotateToByIMU( robot.getImuAngle()-angleOffset);


        mController.driveByEncoders(0.5, -500);



        mController.driveByEncoders(0.5, (int)(56*dist) + 500);


        //spin and pick up

        mController.drive(0.4);
        mController.rotationalModifier(1);
        mController.update();
        mController.sleep(400);
        inDep.closeClaw();
        mController.sleep(200);
        mController.stop();
        inDep.setArmPosition(1000);
        while(opModeIsActive());

    }
}
