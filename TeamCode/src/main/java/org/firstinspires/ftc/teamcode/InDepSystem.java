package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class InDepSystem {

    HardwareRobot robot;
    LinearOpMode linearOpMode;
    Telemetry telemetry;

    int[] levels = {4500, 4100, 3500};
    final int PICKUP_LEVEL = 0;
    final int BARRIER_LEVEL = 400;

    public InDepSystem(HardwareRobot hwRobot, OpMode opMode) {
        robot = hwRobot;
        telemetry = opMode.telemetry;

        if (opMode instanceof LinearOpMode) {
            linearOpMode = (LinearOpMode) opMode;
        }
    }

    public void closeClaw() {
        robot.clawLeft.setPosition(1.0);
        robot.clawRight.setPosition(0.0);
    }

    public void openClaw() {
        robot.clawLeft.setPosition(0.7);
        robot.clawRight.setPosition(0.3);
    }

    public void setClawPosition(double left, double right) {
        robot.clawLeft.setPosition(left);
        robot.clawRight.setPosition(right);
    }

    public void waitForArm() {
        while (opModeIsActive() && robot.clawArm.isBusy()) ;
    }

    public void liftToHubLevel(int level) {
        if (level > levels.length - 1 || level < 0) {
            telemetry.log().add("Level passed to liftToHubLevel was %i when it should be > 0 and < %i", level, levels.length - 1);
            telemetry.update();
            return;
        }
        robot.clawArm.setTargetPosition(levels[level]);
    }

    public void liftToPickup() {
        robot.clawArm.setTargetPosition(PICKUP_LEVEL);
    }

    public void liftToBarrier() {
        robot.clawArm.setTargetPosition(BARRIER_LEVEL);
    }

    public void setArmPosition(int encoder) {
        robot.clawArm.setTargetPosition(encoder);
    }

    public void waitForClaw() {
        long startTimer = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTimer < 1000) ;
    }

    public boolean opModeIsActive() {
        return linearOpMode == null || linearOpMode.opModeIsActive();
    }
}
