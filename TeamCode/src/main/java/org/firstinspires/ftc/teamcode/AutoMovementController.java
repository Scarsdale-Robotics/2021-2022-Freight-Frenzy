package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoMovementController extends MovementController {

    LinearOpMode opMode;

    public AutoMovementController(HardwareRobot r, Telemetry t, LinearOpMode linearOpMode) {
        super(r, t);
        opMode = linearOpMode;
    }

    public void lift(int level) {
        liftAsync(level);
        while(opMode.opModeIsActive() && robot.elevatorCable.isBusy());
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

        long time = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - time < 50);

        stop();
        update();
    }
}
