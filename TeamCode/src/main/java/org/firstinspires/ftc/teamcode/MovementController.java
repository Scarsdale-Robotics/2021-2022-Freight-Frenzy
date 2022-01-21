package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;
import java.util.TimerTask;

public class MovementController {
    private final HardwareRobot robot;
    private final Telemetry telemetry;

    private double limit = 1.0;
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;
    public int levelArray[] = {0, 300, 3300};

    public MovementController(HardwareRobot r, Telemetry t) {
        robot = r;
        telemetry = t;
    }

    public void setLimit(double limit) {
        this.limit = limit;
    }

    public void rotate(double power) {
        leftFrontPower = power;
        leftBackPower = power;
        rightFrontPower = -power;
        rightBackPower = -power;
    }

    public void joystickMovement(double x, double y) {
        double maxValue = Math.abs(x) + Math.abs(y);
        maxValue = (maxValue < 1) ? 1 : maxValue;

        x = -x;

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


    public void strafe(double power) {
        leftFrontPower = power;
        leftBackPower = -power;
        rightFrontPower = -power;
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

        telemetry.addData("Rotation", rotation);
        telemetry.addData("Calculated Rotational Power", calculatedRotationalPower);
    }

    //WARNING: Will complete function before it stops
    public void brakeForAsync(int ms) {
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

    public void brake() {
        brakeFor(50);
    }

    public void brakeFor(int ms) {
        leftFrontPower *= -1;
        leftBackPower *= -1;
        rightFrontPower *= -1;
        rightBackPower *= -1;

        update();

        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            telemetry.log().add("Thread interrupted while braking! This should never happen.");
        }

        stop();
        update();
    }

    public void stop() {
        leftFrontPower = 0;
        leftBackPower = 0;
        rightFrontPower = 0;
        rightBackPower = 0;

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

    public void closeCLaw() {
        robot.clawLeft.setPosition(1.0);
        robot.clawRight.setPosition(0.0);
    }

    public void openClaw() {
        robot.clawLeft.setPosition(0.7);
        robot.clawRight.setPosition(0.3);
    }


    //Blocking movement calls
    public void driveForwardEncoders(float power, int encoderSteps) {
        int[] startEncoders = {robot.leftBack.getCurrentPosition(), robot.rightBack.getCurrentPosition()};
        int[] deltaEncoders = {0, 0};

        if (Math.signum(power) != Math.signum(encoderSteps)) power *= -1;

        joystickMovement(0, power);
        while (Math.abs((deltaEncoders[0] + deltaEncoders[1])/2) < Math.abs(encoderSteps)){
            deltaEncoders[0] = robot.leftBack.getCurrentPosition() - startEncoders[0];
            deltaEncoders[1] = robot.rightBack.getCurrentPosition() - startEncoders[1];
        }
        joystickMovement(0, 0);
    }


    public void lift(int level) {
        robot.clawArm.setTargetPosition(levelArray[level]);
    }


}