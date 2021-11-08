package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class HardwareRobot {
    //Motors
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor spinThing = null;


    public DcMotor clawArm = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;

    public Rev2mDistanceSensor frontDist = null;
    public Rev2mDistanceSensor leftDist = null;
    public Rev2mDistanceSensor rightDist = null;
    public Rev2mDistanceSensor backDist = null;



    public BNO055IMU imu = null;


    HardwareMap hwMap;

    public HardwareRobot(HardwareMap map) {
        hwMap = map;
        init();
    }

    private void init()
    {
//        imu = hwMap.get(BNO055IMU.class, "imu");
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu.initialize(parameters);

        // match motors to phone labels
        leftFront = hwMap.dcMotor.get("leftFront");
        leftBack = hwMap.dcMotor.get("leftBack");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightBack = hwMap.dcMotor.get("rightBack");

        spinThing = hwMap.dcMotor.get("spinThing");




        //claw arm
        clawArm = hwMap.dcMotor.get("clawArm");
        leftClaw = hwMap.servo.get("leftClaw");
        rightClaw = hwMap.servo.get("rightClaw");
        rightClaw.scaleRange(-1, 1);
        leftClaw.scaleRange(-1, 1);



        // sensors
        frontDist = hwMap.get(Rev2mDistanceSensor.class, "frontDist");
        backDist = hwMap.get(Rev2mDistanceSensor.class, "backDist");
        leftDist = hwMap.get(Rev2mDistanceSensor.class, "leftDist");
        rightDist = hwMap.get(Rev2mDistanceSensor.class, "rightDist");


        // set direction of motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        spinThing.setDirection(DcMotor.Direction.REVERSE);


        // set power for dc motors and position of servos
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        spinThing.setPower(0);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinThing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // set motor modes
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinThing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}