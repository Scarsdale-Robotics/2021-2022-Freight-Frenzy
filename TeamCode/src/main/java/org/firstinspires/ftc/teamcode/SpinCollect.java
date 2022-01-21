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

        //Starting position: facing straight towards the pile, claw open and down
        //Initialize & start
        mController.lift(0);
        mController.openClaw();

        // get close to wall
        mController.drive(1);
        mController.update();
        while (opModeIsActive() && robot.frontDist.getDistance(DistanceUnit.INCH) > 18);

        //position: facing wall, 10 inches away
        //spin 90 degrees left while closing claw
        double startingDegrees = robot.imu.getAngularOrientation().firstAngle;
        mController.closeCLaw();
        mController.rotate(1);
        mController.update();
        while (opModeIsActive() && robot.imu.getAngularOrientation().firstAngle-startingDegrees > -90);

        //
        // pin back into a slightly angled position while lifting claw
        mController.lift(2);
        mController.rotate(-1);
        mController.update();
        while (opModeIsActive() && robot.imu.getAngularOrientation().firstAngle-startingDegrees < 25);

        //drive to the tower
        mController.drive(-1);
        mController.update();
        while(opModeIsActive() && robot.frontDist.getDistance(DistanceUnit.INCH)<33.5);

        //drop off the cargo
        mController.stop();
        mController.openClaw();
        mController.update();
    }
}

