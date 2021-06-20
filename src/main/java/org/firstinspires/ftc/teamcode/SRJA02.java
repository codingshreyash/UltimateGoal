package org.firstinspires.ftc.teamcode;

//imports:

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "SRJA02", group = "")
@Disabled
public class SRJA02 extends LinearOpMode {

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

        // reset encoder counts kept by motors.
        //flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run forward for 2500 encoder counts.
        flMotor.setTargetPosition(2500);
        frMotor.setTargetPosition(2500);
        blMotor.setTargetPosition(2500);
        brMotor.setTargetPosition(2500);

        // set motors to run to target encoder position and stop
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 50% power. Movement will start. Sign of power is
        // ignored as sign of target encoder position controls direction when
        // running to position.

        flMotor.setPower(0.50);
        frMotor.setPower(0.50);
        blMotor.setPower(0.50);
        brMotor.setPower(0.50);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && flMotor.isBusy()) {
            telemetry.addData("encoder-fwd-left", flMotor.getCurrentPosition() + "  busy=" + flMotor.isBusy());
            telemetry.addData("encoder-fwd-right", frMotor.getCurrentPosition() + "  busy=" + frMotor.isBusy());
            telemetry.addData("encoder-bck-left", blMotor.getCurrentPosition() + "  busy=" + blMotor.isBusy());
            telemetry.addData("encoder-bck-right", brMotor.getCurrentPosition() + "  busy=" + brMotor.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        flMotor.setPower(0.0);
        frMotor.setPower(0.0);
        blMotor.setPower(0.0);
        brMotor.setPower(0.0);

        // wait 5 sec to you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5) {
            telemetry.addData("encoder-fwd-left-end", flMotor.getCurrentPosition());
            telemetry.addData("encoder-fwd-right-end", frMotor.getCurrentPosition());
            telemetry.addData("encoder-bck-left-end", blMotor.getCurrentPosition());
            telemetry.addData("encoder-bck-right-end", brMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // From current position back up to starting point. In this example instead of
        // having the motor monitor the encoder we will monitor the encoder ourselves.

        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flMotor.setTargetPosition(0);
        frMotor.setTargetPosition(0);
        blMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);

        // Power sign matters again as we are running without encoder.
        flMotor.setPower(-0.50);
        frMotor.setPower(-0.50);
        blMotor.setPower(-0.50);
        brMotor.setPower(-0.50);

        while (opModeIsActive() && flMotor.getCurrentPosition() > flMotor.getTargetPosition()) {
            telemetry.addData("encoder-back-left", flMotor.getCurrentPosition());
            telemetry.addData("encoder-back-right", frMotor.getCurrentPosition());
            telemetry.addData("encoder-back-left", blMotor.getCurrentPosition());
            telemetry.addData("encoder-back-right", brMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.
        flMotor.setPower(0.0);
        frMotor.setPower(0.0);
        blMotor.setPower(0.0);
        brMotor.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5) {
            telemetry.addData("encoder-back-left-end", flMotor.getCurrentPosition());
            telemetry.addData("encoder-back-right-end", frMotor.getCurrentPosition());
            telemetry.addData("encoder-back-left-end", blMotor.getCurrentPosition());
            telemetry.addData("encoder-back-right-end", brMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}