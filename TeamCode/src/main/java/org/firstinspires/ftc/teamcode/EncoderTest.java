package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@Autonomous(name="EncoderTest")
public class EncoderTest extends LinearOpMode {

    MovementController mController;
    HardwareRobot robot;
    InDepSystem inDep;
    ArrayList<Double> testArray = new ArrayList<Double>(); //holds distance from the wall detected of the robot
    ArrayList<Double> difArray = new ArrayList<Double>(); // the n+1th position holds the difference between pos n and n+1 in testArray

    @Override
    public void runOpMode() {

        robot = new HardwareRobot(hardwareMap);
        mController = new MovementController(robot, this);
        inDep = new InDepSystem(robot, this);

        double testData;
        double difData;
        testData = robot.frontDist.getDistance(DistanceUnit.CM);
        testArray.add(testData);
        difArray.add(0.0);
        for (int i=0;i<=4; i++) {
            mController.driveByEncoders(2*((i % 2) - 0.5), 1120);
            testData = robot.frontDist.getDistance(DistanceUnit.CM);
            difData = testData - testArray.get(testArray.size()-1);
            testArray.add(testData);
            difArray.add(difData);
        }
    }
}