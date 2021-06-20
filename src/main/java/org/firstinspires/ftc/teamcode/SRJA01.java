package org.firstinspires.ftc.teamcode;

//imports:
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "SRJA01", group = "")
@Disabled
public class SRJA01 extends LinearOpMode {

    //motors
    private DcMotor flMotor; //front left motor
    private DcMotor frMotor; //front Right motor
    private DcMotor blMotor; //back left motor
    private DcMotor brMotor; //back Right motor

    @Override
    public void runOpMode() throws InterruptedException {
        flMotor = hardwareMap.dcMotor.get("flMotor");
        frMotor = hardwareMap.dcMotor.get("frMotor");
        blMotor = hardwareMap.dcMotor.get("blMotor");
        brMotor = hardwareMap.dcMotor.get("brMotor");

        // initialization here.
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        if (opModeIsActive()) {

            frMotor.setPower(0.5);
            flMotor.setPower(0.5);
            brMotor.setPower(0.5);
            blMotor.setPower(0.5);

            sleep(1500);

            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);

//            brMotor.setPower(0);
//            blMotor.setPower(0);
//            brMotor.setPower(-1);
//            blMotor.setPower(1);

//            sleep(500);
        }

    }
}
