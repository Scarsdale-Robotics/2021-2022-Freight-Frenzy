package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static java.lang.Math.min;

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

        //spin 90 degrees right while going forward and closing claw
        mController.drive(0.25);
        inDep.closeClaw();
        double curAngle = robot.getImuAngle();
        double goalAngle = curAngle + 80;
        double rotateAngle;
        while (curAngle<goalAngle){
            rotateAngle = min(1.0,(goalAngle-curAngle)/80 + 0.2);
            mController.rotationalModifier(rotateAngle*1);
            curAngle = robot.getImuAngle();
            mController.update();
        }

        // turn back
        mController.drive(-0.25);
        curAngle = robot.getImuAngle();
        goalAngle = curAngle-80;
        while(curAngle>goalAngle){
            rotateAngle = min(1.0,(curAngle-goalAngle)/80 + 0.2);
            mController.rotationalModifier(-rotateAngle*1);
            curAngle = robot.getImuAngle();
            mController.update();

        }

        //drive to the tower, arm up
        inDep.liftToHubLevel(0);
        mController.driveByTime(-1, 2000);

        //drop off the cargo
        /*mController.rotateToByIMU(.2, -45);
        inDep.openClaw();*/
    }
}