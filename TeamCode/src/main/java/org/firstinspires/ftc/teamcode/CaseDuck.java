package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "CaseDuck1")
public class CaseDuck extends LinearOpMode {
    public void runOpMode() {
        waitForStart();
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);

        //move to shipping hub

        boolean atHub = false;
        while (opModeIsActive() && !atHub) {
            double backDist = robot.backDist.getDistance(DistanceUnit.INCH);
            double leftDist = robot.leftDist.getDistance(DistanceUnit.INCH);
            int moveRight = 0;
            int moveForward = 0;
            if (backDist <= 13) moveForward = 1;
            if (leftDist <= 45) moveRight = 1;
            if (backDist > 13 && leftDist > 45) atHub = true;
            mController.joystickMovement(moveRight, moveForward);
            mController.update();
        }

        //move back to wall
        //does this work vv
        boolean atWall = false;
        while (opModeIsActive() && !atWall) {
            double backDist = robot.backDist.getDistance(DistanceUnit.INCH);
            double leftDist = robot.leftDist.getDistance(DistanceUnit.INCH);
            double moveBack = 0;
            double moveLeft = 0;
            if (backDist >= 6) moveBack = -0.5;
            if (leftDist >= 6) moveLeft = -0.5;
            if(backDist < 6 && leftDist < 6) atWall = true;
            mController.joystickMovement(moveLeft, moveBack);
            mController.update();

            //should work
        } /*
        while (opModeIsActive() && robot.backDist.getDistance(DistanceUnit.INCH) >= 6) {
            mController.drive(-.2);
            mController.update();

        }
        //move left to carousel
        while (opModeIsActive() && robot.leftDist.getDistance(DistanceUnit.INCH) >= 6) {
            mController.strafe(-0.2);
            mController.update();

        }
        */
    }
}

