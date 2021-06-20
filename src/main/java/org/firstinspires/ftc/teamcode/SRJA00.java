/***************************************************************************************
 *    Team: Tech-Knights
 *    Authors: Shreyash Ranjan
 *    Mentors: Chetan Ranjan, Tejas Priyadarshi
 *    Organization: FIRST Tech Challenge
 *    Version: 1.0.5
 *    Release Date: 12-28-2020
 ***************************************************************************************/

package org.firstinspires.ftc.teamcode;

//imports:
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "SRJA00", group = "")
@Disabled

public class SRJA00 extends OpMode {

    //motors
    private DcMotor flMotor; //front left motor
    private DcMotor frMotor; //front Right motor
    private DcMotor blMotor; //back left motor
    private DcMotor brMotor; //back Right motor
    private BNO055IMU imu; //Gyro I2C port 0, inertial measurement unit
    Orientation threeAngles;
    double lastAngle;
    double globalAngle;
    boolean droveAlready;
    double rotateAngle = 90;
    boolean rotatedAlready;
    Orientation lastAngles = new Orientation();

    @Override
    public void init() {
        flMotor = hardwareMap.dcMotor.get("flMotor");
        frMotor = hardwareMap.dcMotor.get("frMotor");
        blMotor = hardwareMap.dcMotor.get("blMotor");
        brMotor = hardwareMap.dcMotor.get("brMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");

        threeAngles = new Orientation();
        droveAlready = false;
        rotatedAlready = false;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        // initialization here.
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);


            //reset encoder counts kept by motors.
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        if(droveAlready == false) {
            driveDistance();
        } else if (rotatedAlready == false) {

            flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            if(Math.abs(getAngle()) < rotateAngle) {
                flMotor.setPower(0.5);
                blMotor.setPower(0.5);
                frMotor.setPower(0.5);
                brMotor.setPower(0.5);
            } else {
                flMotor.setPower(0);
                blMotor.setPower(0);
                frMotor.setPower(0);
                brMotor.setPower(0);
                rotatedAlready = true;
            }
        }






        telemetry.addData("Current Position ", flMotor.getCurrentPosition());

        threeAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        double currentAngle = threeAngles.firstAngle;
        telemetry.addData("Current Angle ", getAngle());
        telemetry.update();

    }

//    private double getAngle() {
//        threeAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
//        double currentAngle = threeAngles.firstAngle;
//        double deltaAngle = currentAngle - lastAngle;
//        if (deltaAngle < -180) {
//            deltaAngle += 360;
//        }
//        else if (deltaAngle > 180) {
//            deltaAngle -= 360;
//        }
//        globalAngle += deltaAngle;
//        lastAngle = currentAngle;
//        return globalAngle;
//    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180) {
            deltaAngle += 360;
        }
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    private void driveDistance() {
        // set motors to run forward for 2500 encoder counts.
        flMotor.setTargetPosition(2500);
        frMotor.setTargetPosition(2500);
        blMotor.setTargetPosition(2500);
        brMotor.setTargetPosition(2500);

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(flMotor.getCurrentPosition()) < flMotor.getTargetPosition()) {
            flMotor.setPower(0.5);
            frMotor.setPower(0.5);
            blMotor.setPower(0.5);
            brMotor.setPower(0.5);
        } else{
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
            droveAlready = true;
        }
    }
}
