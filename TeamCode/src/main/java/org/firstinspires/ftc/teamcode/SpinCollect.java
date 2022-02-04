package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="SpinCollect")
public class SpinCollect extends LinearOpMode {

    MovementController mController;
    HardwareRobot robot;
    InDepSystem inDep;

    @Override
    public void runOpMode() {

        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, this);
        inDep = new InDepSystem(robot, this);

        // Starting position: facing wall, claw semi-down and open
        // Init the claw
        inDep.liftToPickup();
        inDep.waitForArm();

        //position: facing wall, claw down and open
        //spin 90 degrees right while closing claw
        mController.rotateToByIMU(.5, 180);
        inDep.closeClaw();
        mController.update();

        // turn back
        mController.rotateToByIMU(.5, 90);

        //drive to the tower, arm up
        inDep.liftToHubLevel(0);
        mController.driveByTime(-1, 2000);
        mController.update();

        //drop off the cargo
        mController.rotateToByIMU(.2, -45);
        mController.update();
        inDep.openClaw();
        }
    }