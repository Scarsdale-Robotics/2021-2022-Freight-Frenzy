package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="SpinCollect")
public class SpinCollect extends LinearOpMode {

    MovementController mController;
    HardwareRobot robot;
    InDepSystem inDep;

    public void setPowerForwardTurn(double drivePower, double turnPower){

    }

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
        mController.driveByEncoders(0.5,800);
        inDep.closeClaw();
        double initAngle = robot.getImuAngle();
        double curAngle = initAngle;
        while (curAngle<initAngle+90){
            mController.rotationalModifier((initAngle+90-curAngle)/90);
            curAngle = robot.getImuAngle();
            mController.update();
        }

        // turn back
        mController.driveByEncoders(-0.5,800);
        initAngle = robot.getImuAngle();
        curAngle = initAngle;
        while(curAngle>initAngle-90){
            mController.rotationalModifier(-(curAngle-initAngle+90)/90);
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