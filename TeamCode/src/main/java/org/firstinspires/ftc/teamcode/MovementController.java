package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Timer;
import java.util.TimerTask;

public class MovementController {
    private final HardwareRobot robot;
    private final Telemetry telemetry;
    private LinearOpMode linearOpMode = null;

    private double limit = 1.0;
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;

    public MovementController(HardwareRobot hwRobot, OpMode opMode) {
        robot = hwRobot;
        telemetry = opMode.telemetry;

        if (opMode instanceof LinearOpMode) {
            linearOpMode = (LinearOpMode) opMode;
        }

    }

    public void joystickMovement(double x, double y) {
        double maxValue = Math.abs(x) + Math.abs(y);
        maxValue = (maxValue < 1) ? 1 : maxValue;


        leftFrontPower = (y / maxValue) - (x / maxValue);
        leftBackPower = (y / maxValue) + (x / maxValue);
        rightFrontPower = (y / maxValue) + (x / maxValue);
        rightBackPower = (y / maxValue) - (x / maxValue);
    }

    public void drive(double power) {
        leftFrontPower = power;
        leftBackPower = power;
        rightFrontPower = power;
        rightBackPower = power;
    }

    public void rotationalModifier(double rotationPower) {
        double x = -rotationPower * 0.85;

        rightFrontPower = ((Math.abs(rightFrontPower) > 1) ? Math.signum(rightFrontPower) : rightFrontPower);
        rightBackPower = ((Math.abs(rightBackPower) > 1) ? Math.signum(rightBackPower) : rightBackPower);

        rightFrontPower += x;
        rightBackPower += x;

        if (Math.abs(rightFrontPower) > 1) {
            rightBackPower += rightFrontPower - Math.signum(rightFrontPower);
        } else if (Math.abs(rightBackPower) > 1) {
            rightFrontPower += rightBackPower - Math.signum(rightBackPower);
        }

        leftFrontPower = ((Math.abs(leftFrontPower) > 1) ? Math.signum(leftFrontPower) : leftFrontPower);
        leftBackPower = ((Math.abs(leftBackPower) > 1) ? Math.signum(leftBackPower) : leftBackPower);

        leftFrontPower -= x;
        leftBackPower -= x;

        if (Math.abs(leftFrontPower) > 1) {
            leftBackPower += leftFrontPower - Math.signum(leftFrontPower);
        } else if (Math.abs(leftBackPower) > 1) {
            leftFrontPower += leftBackPower - Math.signum(leftBackPower);
        }
    }

    public void rotateInPlace(double power) {
        rightBackPower = -power;
        rightFrontPower = -power;
        leftBackPower = power;
        leftFrontPower = power;
    }


    public void rotationalForward(double power, double rotation) {
        // rotational power calculated from current heading (in degrees)
        double calculatedRotationalPower = rotation;

        if (270 <= rotation && rotation < 360) {
            calculatedRotationalPower = -(calculatedRotationalPower - 360) / 90.0;
        } else if (0 < rotation && rotation <= 90) {
            calculatedRotationalPower /= -90;
        }

        double COEFFICIENT = 10 / Math.PI;
        calculatedRotationalPower = -0.8 * Math.atan(COEFFICIENT * calculatedRotationalPower);

        if (-1 > calculatedRotationalPower || calculatedRotationalPower > 1) {
            calculatedRotationalPower = Math.signum(calculatedRotationalPower);
        }

        drive(power);
        rotationalModifier(calculatedRotationalPower);
    }

    public void brakeFor(int ms) {
        leftFrontPower *= -1;
        leftBackPower *= -1;
        rightFrontPower *= -1;
        rightBackPower *= -1;

        update();

        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                stop();
                update();
            }
        }, ms);
    }

    public void stop() {
        leftFrontPower = 0;
        leftBackPower = 0;
        rightFrontPower = 0;
        rightBackPower = 0;
        update();
    }

    public void update() {
        if (limit <= 1) {
            leftFrontPower *= limit;
            leftBackPower *= limit;
            rightFrontPower *= limit;
            rightBackPower *= limit;
        } else {
            leftFrontPower /= limit;
            leftBackPower /= limit;
            rightFrontPower /= limit;
            rightBackPower /= limit;
        }

        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightFront.setPower(-rightFrontPower);
        robot.rightBack.setPower(-rightBackPower);
    }

    //Blocking movement calls
    public void driveByEncoders(double power, int encoderSteps) {
        int lStart = Math.abs(robot.leftBack.getCurrentPosition());
        int rStart = Math.abs(robot.rightBack.getCurrentPosition());

        int deltaRight = 0;
        int deltaLeft = 0;
        joystickMovement(0, power);
        encoderSteps = Math.abs(encoderSteps);
        update();
        while (opModeIsActive() && (deltaLeft + deltaRight) / 2 < encoderSteps) {
            deltaLeft = Math.abs(Math.abs(robot.leftBack.getCurrentPosition()) - lStart);
            deltaRight = Math.abs(Math.abs(robot.rightBack.getCurrentPosition()) - rStart);
//            telemetry.addData("deltaLeft: ", deltaLeft);
//            telemetry.addData("deltaRight: ", deltaRight);
//            telemetry.addData("leftBack: ", robot.leftBack.getCurrentPosition());
//            telemetry.addData("rightBack: ", robot.rightBack.getCurrentPosition());
//            telemetry.addData("leftFront: ", robot.leftFront.getCurrentPosition());
//            telemetry.addData("rightFront: ", robot.rightFront.getCurrentPosition());
//            telemetry.update();
        }
        stop();
        update();
    }

    public void driveByDistance(double power, Rev2mDistanceSensor distanceSensor, double inches, boolean approaching) {
        drive(power);
        update();

        double distance;
        if (approaching) { //drive until distance is smaller than inches
            distance = 9999999;
            while (opModeIsActive() && distance > inches) {
                distance = robot.getDistance(distanceSensor);
            }

        } else { //drive until distance is larger than inches
            distance = 0;

            while (opModeIsActive() && distance < inches) {
                distance = distanceSensor.getDistance(DistanceUnit.INCH);
            }
        }

        stop();
        update();
    }

    public void driveByTime(double power, long millis) { //Please never use
        long startTime = System.currentTimeMillis();
        drive(power);
        update();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < millis) ;
        stop();
        update();
    }

    public void rotateToByIMU(float angle) {
        double power = 0;


        while (opModeIsActive() && Math.abs(angle - robot.getImuAngle()) > 2.5) {
            float delta = Math.abs(angle - robot.getImuAngle());
            power = Math.pow(delta, 1.0 / 3) / 10;

            if (power < 0.2) power = 0.2;
            else if (power > 0.4) power = 1;
            if (angle > robot.getImuAngle()) {
                power *= -1;
            }
            rotateInPlace(power);
            update();
        }

        stop();
        update();
    }

    public void sleep(long millis) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < millis) ;
    }

    public boolean opModeIsActive() {
        return linearOpMode == null || linearOpMode.opModeIsActive();
    }

    public void pivotOnRight(double power) {
        leftFrontPower = power;
        leftBackPower = power;
        rightBackPower = 0;
        rightFrontPower = 0;
    }

    public void pivotOnLeft(double power) {
        leftFrontPower = 0;
        leftBackPower = 0;
        rightBackPower = power;
        rightFrontPower = power;
    }
}