package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "DistanceThing")
public class DistanceThing extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MovementController mController = new MovementController(robot, telemetry);
        while(opModeIsActive()) {
            if(robot.frontDist.getDistance(DistanceUnit.CM) >= 50){
                mController.drive(1);
            }
            else{
                mController.stop();
            }
            mController.update();

        }

    }
}