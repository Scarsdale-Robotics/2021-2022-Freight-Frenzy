package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "posDemo_auto")
public class posDemo extends LinearOpMode {

    public void getToPos(double outerL, double innerL, double heading) {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);
        while(robot.imu.getAngularOrientation() < -90) {
            mController.rotate(1);
            mController.update();
        }
        mController.stop();
        mController.update();
        while(robot.backDist.getDistance(DistanceUnit.INCH) < innerL) {
            mController.drive(0.2);
            mController.update();
        }
        while(robot.imu.getAngularOrientation() != 180) {
            mController.rotate(1);
            mController.update();
        }
        while(robot.backDist.getDistance(DistanceUnit.INCH) < innerL) {
            mController.drive(0.2);
            mController.update();
        }
        while(robot.imu.getAngularOrientation() != heading) {
            mController.rotate(1);
            mController.update();
        }




    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
