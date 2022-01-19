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

        int duckPos = duckDetector.getDuckPosition();
        for (int i = 0; i < 50; i++) {
            if (duckPos != -1) {
                duckPos = duckDetector.getDuckPosition();
                break;
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

        telemetry.addData("reeee: ", duckPos);

        telemetry.update();

        mController.stop();
        mController.update();

        robot.clawArm.setTargetPosition(1600);


        while (opModeIsActive() && robot.clawArm.isBusy()) ;

        float startAngle = robot.imu.getAngularOrientation().firstAngle;
        mController.rotate(-0.5);
        mController.update();
        while (opModeIsActive() && startAngle - robot.imu.getAngularOrientation().firstAngle > -15)
            ;
        mController.stop();
        mController.update();


        int levels[] = {4800, 4050, 3500};
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


        mController.openClaw();
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTimer < 2000) ;

        robot.clawRight.setPosition(1);
        robot.clawLeft.setPosition(-1);


        mController.joystickMovement(0, 0.7);
        mController.update();
        startTimer = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTimer < 1000) ;
        mController.stop();
        mController.update();


        startAngle = robot.imu.getAngularOrientation().firstAngle;
        mController.rotate(0.5);
        mController.update();
        while (opModeIsActive() && startAngle - robot.imu.getAngularOrientation().firstAngle < 105)
            ;
        mController.stop();
        mController.update();

    }
}
