package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class HardwareRobot {
    //Motors
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;

    public DcMotor duckSpinLeft = null;
    public DcMotor duckSpinRight = null;

    public Rev2mDistanceSensor frontDist = null;
    public Rev2mDistanceSensor highFrontDist = null;

    public Rev2mDistanceSensor leftDist = null;
    public Rev2mDistanceSensor rightDist = null;
    public Rev2mDistanceSensor backDist = null;

    public BNO055IMU imu = null;
    public final float startAngle;

    public DcMotor clawArm = null;
    public Servo clawLeft = null;
    public Servo clawRight = null;

    HardwareMap hwMap;

    public HardwareRobot(HardwareMap map) {
        hwMap = map;

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        // match motors to phone labels
        leftFront = hwMap.dcMotor.get("leftFront");
        leftBack = hwMap.dcMotor.get("leftBack");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightBack = hwMap.dcMotor.get("rightBack");

        duckSpinRight = hwMap.dcMotor.get("duckSpinRight");
        duckSpinLeft = hwMap.dcMotor.get("duckSpinLeft");

        clawArm = hwMap.dcMotor.get("clawArm");

        clawLeft = hwMap.servo.get("clawLeft");
        clawRight = hwMap.servo.get("clawRight");
        clawLeft.scaleRange(0, 1);
        clawRight.scaleRange(0, 1);

        // sensors
        frontDist = hwMap.get(Rev2mDistanceSensor.class, "frontDist");
        highFrontDist = hwMap.get(Rev2mDistanceSensor.class, "highFrontDist");

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

        clawArm.setDirection(DcMotor.Direction.REVERSE);

        // set power for dc motors and position of servos
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);


        duckSpinLeft.setPower(0);
        duckSpinRight.setPower(0);
        clawArm.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        duckSpinLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckSpinRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        duckSpinLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckSpinRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawArm.setTargetPosition(0);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawArm.setPower(1);

        Orientation orientation = imu.getAngularOrientation();
        startAngle = AngleUnit.DEGREES.fromUnit(orientation.angleUnit, orientation.firstAngle);
    }

    public float getImuAngle() {
        Orientation orientation = imu.getAngularOrientation();
        return AngleUnit.DEGREES.fromUnit(orientation.angleUnit, orientation.firstAngle) - startAngle;
    }

    public double getDistance(Rev2mDistanceSensor sensor){
        return sensor.getDistance(DistanceUnit.INCH);
    }
}
