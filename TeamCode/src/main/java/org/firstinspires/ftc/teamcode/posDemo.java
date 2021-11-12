package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "posDemo_auto")
public class posDemo extends LinearOpMode {

    public void getToPos(double outerL, double heading) {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);
        while(robot.getImuAngle() < 80 || robot.getImuAngle() > 100) {
            mController.rotate(1);
            mController.update();
        }
        while(robot.backDist.getDistance(DistanceUnit.INCH) < outerL) {
            mController.drive(0.2);
            mController.update();
        }
        while(robot.getImuAngle() < 170 || robot.getImuAngle() > -170) {
            mController.rotate(1);
            mController.update();
        }
        while(robot.backDist.getDistance(DistanceUnit.INCH) < outerL) {
            mController.drive(0.2);
            mController.update();
        }
        while(robot.getImuAngle() < heading - 10 || robot.getImuAngle() > heading + 10) {
            mController.rotate(1);
            mController.update();
        }




    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
