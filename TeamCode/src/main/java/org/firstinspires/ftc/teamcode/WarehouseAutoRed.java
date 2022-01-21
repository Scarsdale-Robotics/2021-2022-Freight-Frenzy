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
        mController.joystickMovement(0, -0.5);
        mController.update();
        while (opModeIsActive() && (robot.frontDist.getDistance(DistanceUnit.INCH) < 5 || robot.frontDist.getDistance(DistanceUnit.CM) > 999)) {
            if (duckPos == -1) {
                duckPos = duckDetector.getDuckPosition();
            }
        }

        //telemetry.addData("position: ", duckPos);

        //telemetry.update();

        mController.stop();
        mController.update();

        robot.clawArm.setTargetPosition(1600);


        while (opModeIsActive() && robot.clawArm.isBusy()) ;

        float startAngle = robot.imu.getAngularOrientation().firstAngle;
        mController.rotate(-0.2);
        mController.update();
        while (opModeIsActive() && startAngle - robot.imu.getAngularOrientation().firstAngle > -32)
            ;
        mController.stop();
        mController.update();


        int levels[] = {5000, 4200, 3500};
        if (duckPos == -1) {
            duckPos = 2;
        }
        robot.clawArm.setTargetPosition(levels[duckPos]);
        while (opModeIsActive() && robot.clawArm.isBusy()) ;

        mController.joystickMovement(0, -0.7);
        mController.update();
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTimer < 1000) ;
        mController.stop();
        mController.update();

        robot.clawLeft.setPosition(0);
        robot.clawRight.setPosition(0);
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTimer < 2000) ;

        if(duckPos == 0){
            int targetPos = 4550;
            robot.clawArm.setTargetPosition(targetPos);
            telemetry.addData("Waiting", null);
            while(Math.abs(robot.clawArm.getCurrentPosition() - targetPos) > 50);
        }


        if(duckPos != 0) {
            robot.clawRight.setPosition(1);
            robot.clawLeft.setPosition(-1);
            startTimer = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - startTimer < 2000) ;
        }

        mController.joystickMovement(0, 0.7);
        mController.update();
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTimer < 1000) ;
        mController.stop();
        mController.update();


        robot.clawArm.setTargetPosition(400);
        mController.rotate(-0.2);
        mController.update();
        while (opModeIsActive() && startAngle - robot.imu.getAngularOrientation().firstAngle > -90)
            ;
        mController.stop();
        mController.update();
        mController.openClaw();


        mController.joystickMovement(0, -0.6);
        mController.update();
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTimer < 500) ;
        mController.stop();
        mController.update();

        sleep(200);

        mController.joystickMovement(0, 1.0);
        mController.update();
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTimer < 1500) ;
        mController.stop();
        mController.update();

    }
}
