package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;
import java.util.TimerTask;

public class MovementController {
    protected final HardwareRobot robot;
    protected final Telemetry telemetry;

    private double limit = 1.0;
    protected double leftFrontPower = 0;
    protected double leftBackPower = 0;
    protected double rightFrontPower = 0;
    protected double rightBackPower = 0;
    private int levelArray[] = {0, 0, 0, 0};

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

        y = -y;
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

        if(270 <= rotation && rotation < 360) {
            calculatedRotationalPower = -(calculatedRotationalPower - 360) / 90.0;
        } else if(0 < rotation && rotation <= 90) {
            calculatedRotationalPower /= -90;
        }

        double COEFFICIENT = 10 / Math.PI;
        calculatedRotationalPower = -0.8 * Math.atan(COEFFICIENT * calculatedRotationalPower);

        if(-1 > calculatedRotationalPower || calculatedRotationalPower > 1) {
            calculatedRotationalPower = Math.signum(calculatedRotationalPower);
        }

        drive(power);
        rotationalModifier(calculatedRotationalPower);

        telemetry.addData("Rotation", rotation);
        telemetry.addData("Calculated Rotational Power", calculatedRotationalPower);
    }

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

    public void stop() {
        leftFrontPower = 0;
        leftBackPower = 0;
        rightFrontPower = 0;
        rightBackPower = 0;

    }

    public void update() {
        if(limit <= 1) {
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

    public void liftAsync(int level){
        robot.elevatorCable.setTargetPosition(levelArray[level] % levelArray.length);
        robot.elevatorCable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elevatorCable.setPower(1);
    }
}