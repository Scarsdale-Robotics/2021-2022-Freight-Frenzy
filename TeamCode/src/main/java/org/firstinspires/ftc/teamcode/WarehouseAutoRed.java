package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="WarehouseRed")
public class WarehouseAutoRed extends LinearOpMode {

    DuckCV duckDetector;

    MovementController mController;
    HardwareRobot robot;

    int elevatorLevel = 0;


    @Override
    public void runOpMode() {

        //Init
        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, telemetry);
        robot.clawArm.setPower(1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        duckDetector = new DuckCV(cameraMonitorViewId);

        waitForStart();
        int duckPos = duckDetector.getDuckPosition();

        telemetry.addData("DuckPos: ", duckPos);
        telemetry.update();

        //move back
        long startTimer = System.nanoTime();

        mController.joystickMovement(0, -0.4);
        robot.clawArm.setTargetPosition(600);
        mController.update();
        while(opModeIsActive() && System.nanoTime() - startTimer < 1 * 1000000000); //seconds to nanoseconds
        mController.stop();
        mController.update();

        //Lower and unfold claw
        mController.openClaw();
        startTimer = System.nanoTime();
        while(opModeIsActive() && System.nanoTime() - startTimer < 2L * 1_000_000_000);
        robot.clawArm.setTargetPosition(15);
        while(opModeIsActive() && robot.clawArm.isBusy());

        //move forward
        mController.joystickMovement(0, 0.4);
        mController.update();
        startTimer = System.nanoTime();
        while(opModeIsActive() && System.nanoTime() - startTimer < 2L * 1_000_000_000);
        mController.stop();
        mController.update();

        //Close Claw
        mController.closeCLaw();
        startTimer = System.nanoTime();
        while(opModeIsActive() && System.nanoTime() - startTimer < 2L * 1_000_000_000);

        //move back
        mController.joystickMovement(0, -0.4);
        mController.update();
        startTimer = System.nanoTime();
        while(opModeIsActive() && System.nanoTime() - startTimer < 0.1 * 1_000_000_000);
        mController.stop();
        mController.update();
        robot.clawArm.setTargetPosition(1600);

        while(opModeIsActive() && Math.abs(robot.clawArm.getCurrentPosition() - 1600) > 100);

        float startAngle = robot.imu.getAngularOrientation().firstAngle;
        mController.rotate(0.5);
        mController.update();
        while(opModeIsActive() && startAngle - robot.imu.getAngularOrientation().firstAngle < 50);
        mController.stop();
        mController.update();


        //move towards the shipping hub\

        startTimer = System.nanoTime();

        if(duckPos == -1){
            duckPos = 0;
        }
        int levels[] = {3600, 3900, 4200};
        robot.clawArm.setTargetPosition(levels[duckPos]);

        mController.joystickMovement(0, 0.5);
        mController.update();
        while(opModeIsActive() && System.nanoTime() - startTimer < 0.3 * 1_000_000_000);
        mController.stop();
        mController.update();



        while(opModeIsActive());



    }
}
