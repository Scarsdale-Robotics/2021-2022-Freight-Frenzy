package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="SpinCollect")
public class SpinCollect extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);

        //Starting position: facing straight towards the pile, claw open and in position
        mController.setPosition(0.5);
        mController.drive(1);
        mController.update();

        while (opModeIsActive() && robot.frontDist.getDistance(DistanceUnit.INCH) > 10) {}

        //position: facing wall
        //spin 180 degrees
        //firstAngle???
        double startingDegrees = robot.imu.getAngularOrientation().firstAngle;
        while (opModeIsActive() && robot.imu.getAngularOrientation().firstAngle-startingDegrees < 180) {
            mController.rotationalForward(1, 0.5);
            mController.update();
        }
        mController.stop();
        mController.update();

    }
}


package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="SpinCollect")
public class SpinCollect extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);

        //Starting position: facing straight towards the pile, claw open and in position
        mController.setPosition(0.5);
        mController.drive(1);
        mController.update();

        while (opModeIsActive() && robot.frontDist.getDistance(DistanceUnit.INCH) > 10) {}

        //position: facing wall
        //spin 180 degrees
        //firstAngle???
        double startingDegrees = robot.imu.getAngularOrientation().firstAngle;
        while (opModeIsActive() && robot.imu.getAngularOrientation().firstAngle-startingDegrees < 180) {
            mController.rotationalForward(1, 0.5);
            mController.update();
        }
        mController.stop();
        mController.update();

    }
}

