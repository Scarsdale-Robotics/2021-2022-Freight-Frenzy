package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "WarehouseRed")
public class WarehouseAutoRed extends LinearOpMode {

    DuckCV duckDetector;

    MovementController mController;
    HardwareRobot robot;

    int elevatorLevel = 0;
    long startTimer;

    @Override
    public void runOpMode() {

        //Init
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, telemetry);
        robot.clawArm.setPower(1);

        waitForStart();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        duckDetector = new DuckCV(cameraMonitorViewId);

        int votes[] = {0, 0, 0};
        int duckPos = -1;
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTimer < 1500)) {
            duckPos = duckDetector.getDuckPosition();
            telemetry.addData("pos: ", duckPos);
//            telemetry.addData("i: ", i);

            telemetry.addData("x: ", duckDetector.duckX);
            telemetry.addData("y: ", duckDetector.duckY);
            telemetry.update();

            if (duckPos != -1) {
                votes[duckPos]++;
            }
        }
        int max = -1;
        duckPos = -1;
        for (int i = 0; i < 3; i++) {
            if (votes[i] >= max) {
                max = (votes[i]);
                duckPos = i;
            }
        }


        //move back
        mController.driveByDistance(-0.5, robot.frontDist, 5, false);

        robot.clawArm.setTargetPosition(1600);
        while (opModeIsActive() && robot.clawArm.isBusy());

        //turn to shipping hub
        mController.rotateToByIMU(-0.2, -32);


        // Set claw arm to correct position by ducklevel
        int levels[] = {5000, 4200, 3500};
        if (duckPos == -1) {
            duckPos = 2;
        }
        robot.clawArm.setTargetPosition(levels[duckPos]);
        while (opModeIsActive() && robot.clawArm.isBusy()) ;


        //drive to alliance shipping hub
        mController.driveByTime(-0.7, 1000);


        //open claw dropping the cube. Delay because of servo latency
        robot.clawLeft.setPosition(0);
        robot.clawRight.setPosition(0);
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTimer < 2000) ;


        //Bottom level uses ramp which requires placing on the ramp then lifting the arm up
        if(duckPos == 0){
            int targetPos = 4550;
            robot.clawArm.setTargetPosition(targetPos);
            while(Math.abs(robot.clawArm.getCurrentPosition() - targetPos) > 50);
        }else { // Other levels need the claw recrossed
            robot.clawRight.setPosition(1);
            robot.clawLeft.setPosition(-1);
            startTimer = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - startTimer < 2000) ;
        }

        //drive away from alliance shipping hub
        mController.driveByTime(0.7, 900);


        //rotate to face the warehouse and lower arm
        robot.clawArm.setTargetPosition(400);
        mController.rotateToByIMU(-0.2, -90);
        robot.openClaw();


        // Drive backwards because there is not enough room accellerate to full speed to get over barriers
        mController.driveByTime(-0.6, 500);
        sleep(200);

        // Drive into the warehouse
        mController.driveByTime(1, 1500);
    }
}
