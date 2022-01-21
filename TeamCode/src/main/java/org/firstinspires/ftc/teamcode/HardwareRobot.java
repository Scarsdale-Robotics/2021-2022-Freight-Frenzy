package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class HardwareRobot {
    //Motors
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor duckSpinLeft = null;
    public DcMotor duckSpinRight = null;


    public DcMotor elevatorCable = null;
    public DcMotor elevatorIntake = null;


    public Servo elevatorDoor = null;

    public Rev2mDistanceSensor frontDist = null;
    public Rev2mDistanceSensor leftDist = null;
    public Rev2mDistanceSensor rightDist = null;
    public Rev2mDistanceSensor backDist = null;

    public BNO055IMU imu = null;


    public DcMotor clawArm = null;
    public Servo clawLeft = null;
    public Servo clawRight = null;



    HardwareMap hwMap;

    public HardwareRobot(HardwareMap map) {
        hwMap = map;
        init();
    }

    private void init()
    {


        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);


        // match motors to phone labels
        leftFront = hwMap.dcMotor.get("leftFront");
        leftBack = hwMap.dcMotor.get("leftBack");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightBack = hwMap.dcMotor.get("rightBack");

        duckSpinRight = hwMap.dcMotor.get("duckSpinRight");
        duckSpinLeft = hwMap.dcMotor.get("duckSpinLeft");


//        elevatorCable = hwMap.dcMotor.get("elevatorCable");
//        elevatorIntake = hwMap.dcMotor.get("elevatorIntake");
//
//        elevatorDoor = hwMap.servo.get("elevatorDoor");


        clawArm = hwMap.dcMotor.get("clawArm");

        clawLeft = hwMap.servo.get("clawLeft");
        clawRight = hwMap.servo.get("clawRight");


//        elevatorDoor.scaleRange(0, 1);
        clawLeft.scaleRange(0, 1);
        clawRight.scaleRange(0, 1);



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
        duckSpinLeft.setDirection(DcMotor.Direction.REVERSE);
        duckSpinRight.setDirection(DcMotor.Direction.REVERSE);

//        elevatorIntake.setDirection(DcMotor.Direction.REVERSE);
//        elevatorCable.setDirection(DcMotor.Direction.REVERSE);
        clawArm.setDirection(DcMotor.Direction.REVERSE);



        // set power for dc motors and position of servos
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        duckSpinLeft.setPower(0);
        duckSpinRight.setPower(0);

//        elevatorCable.setPower(0);
//        elevatorIntake.setPower(0);
        clawArm.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckSpinLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckSpinRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        elevatorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevatorCable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // set motor modes
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckSpinLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckSpinRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        elevatorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        elevatorCable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        clawArm.setTargetPosition(0);
//        elevatorCable.setTargetPosition(0);


        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevatorCable.setMode(DcMotor.RunMode.RUN_TO_POSITION)

    }
    public double getImuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (double)AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
}