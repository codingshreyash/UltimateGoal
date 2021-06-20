package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "SRJA03", group = "")
@Disabled
public class SRJA03 extends LinearOpMode {

    //vision
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AZcrIgv/////AAABmaP0qt2hbUhysK3WEDuYtEhzEo+N3Y5xGqGA8EVyBvrRXzRPwQQtI0cnizK49MGkI9qKkHhPob1A3CW6shK1ahjCB35reqzRgKIcju/S4ebprvElpxMe4qKxU0piRBvKtjvoogVt2YrI35UpPLux6a8pmazDlDidgX8nJfRU4qINGivjlQpKWWNmS4XpVxeDt+FmWHp+9d49PMibjTzPg2UmwIyTHsiVbkBZNpL9qN8QVY3T05rXUQHr1s9y33SzFa59Vt3zueXV0q02Iab9/XxgfJyymaMRDoFIUpOvhOY9mXx1kMr9VzjN25psHo9gr/P6V4SQ22oAFZ82rGzUzTR0MyeWP9KYnTl78IvDMP16";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //motors
    private DcMotor flMotor; //front left motor
    private DcMotor frMotor; //front Right motor
    private DcMotor blMotor; //back left motor
    private DcMotor brMotor; //back Right motor

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        initDriveMotors();

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                detectImage();
//                testEncoder();
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initDriveMotors() {
        //-------------------START MOTOR INITIALIZATION
        flMotor = hardwareMap.dcMotor.get("flMotor");
        frMotor = hardwareMap.dcMotor.get("frMotor");
        blMotor = hardwareMap.dcMotor.get("blMotor");
        brMotor = hardwareMap.dcMotor.get("brMotor");

        // initialization here.
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // reset encoder counts kept by motors.
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        //-----------------END MOTOR INITIALIZATION

    }

    private void detectImage() {
        if(tfod != null) {
            List<Recognition> recognitions = tfod.getRecognitions();
            int closestConfidence = Integer.MIN_VALUE;
            if(recognitions.size() > 0) {
                Recognition closestRecognition = recognitions.get(0);
                telemetry.addData("# of objects detected", recognitions.size());
                for (Recognition recognition : recognitions) {
                    if (recognition.getConfidence() > closestConfidence) {
                        closestRecognition = recognition;
                    }
                }
                String recognitionLabel = closestRecognition.getLabel();
                telemetry.addData("closest object", recognitionLabel);
                telemetry.update();
            } else {
                telemetry.addData("closest object", "Nothing");

            }

        }
    }

    private void testEncoder() {
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
