/***************************************************************************************
 *    Team: Tech-Knights
 *    Authors: Shreyash Ranjan
 *    Mentors: Chetan Ranjan, Tejas Priyadarshi
 *    Organization: FIRST Tech Challenge
 *    Version: 1.0.5
 *    Release Date: 12-28-2020
 ***************************************************************************************/

package org.firstinspires.ftc.teamcode; //Package class

/* IMPORT CLASSES NEEDED FOR FTC*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/* TELE-OP CLASS*/
@TeleOp(name = "SRJT06", group = "Opmode")
@Disabled
public class SRJT06 extends OpMode{
    // MOTORS
    private DcMotor flMotor; //front left motor PORT: 3 CH
    private DcMotor frMotor; //front right motor PORT:2 CH
    private DcMotor blMotor; //back left motor PORT:1 CH
    private DcMotor brMotor; //back right motor PORT:0 CH
    private DcMotor intakeOne;  //intake motor PORT:0 EH
    private DcMotor intakeTwo;  //intake motor PORT:1 EH
    private BNO055IMU imu2; //Gyro I2C port 0, inertial measurement unit
//    private Gyroscope imu1;

    // VARIABLES FOR DIRECTIONS
    private double forwardPower = 1;
    private double rotationPower = 1;
    private double strafePower = 1;

    @Override
    public void init() {
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        intakeOne = hardwareMap.get(DcMotor.class, "intakeOne");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwo");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu2.initialize(parameters);

//        imu1 = hardwareMap.get(Gyroscope.class, "imu 1");

        // INITIALIZE COMPONENTS
        //1208flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        //1208frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        //1208blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        //1208brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        //sMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start() {
        // Set motor power to 0
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
        // Update Telemetry
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive();
        telemetry.addData("Gyro Angle", imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
//        telemetry.addData("Gyro Angle", imu1);

    }

    public void stop() {

        // Set motor power to 0

        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);

        // Update Telemetry
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    // DRIVE TRAIN CODE
    public void drive() {
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);
        double xValue = gamepad1.right_stick_x * -rotationPower;
        double yValue = gamepad1.left_stick_y * forwardPower;
        double rValue = (gamepad1.left_trigger - gamepad1.right_trigger) * strafePower;

        double flPower = yValue + xValue + rValue;
        double frPower = yValue - xValue - rValue;
        double blPower = yValue + xValue - rValue;
        double brPower = yValue - xValue + rValue;

        telemetry.addData("Y Value", yValue);

        flMotor.setPower(Range.clip(Math.signum(flPower) * Math.pow(flPower, 2), -1.0, 1.0));
        frMotor.setPower(Range.clip(Math.signum(frPower) * Math.pow(frPower, 2), -1.0, 1.0));
        blMotor.setPower(Range.clip(Math.signum(blPower) * Math.pow(blPower, 2), -1.0, 1.0));
        brMotor.setPower(Range.clip(Math.signum(brPower) * Math.pow(brPower, 2), -1.0, 1.0));


        intakeOne.setPower(-1);

        intakeTwo.setPower(.75);


    }
}


