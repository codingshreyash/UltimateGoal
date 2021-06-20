/***************************************************************************************
 *    Team: Tech-Knights
 *    Authors: Shreyash Ranjan
 *    Mentors: Chetan Ranjan, Tejas Priyadarshi
 *    Organization: FIRST Tech Challenge
 *    Version: 20.0.0
 *    Release Date: 03-31-2021
 ***************************************************************************************/
package org.firstinspires.ftc.teamcode;
/*
 * imports
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "SRJT20", group = "Opmode")
public class SRJT20 extends OpMode {

    /*
     * Motors:
     * flMotor-Control Hub-Port: 3
     * frMotor-Control Hub-Port: 2
     * blMotor-Control Hub-Port: 1
     * brMotor-Control Hub-Port: 0
     * sMotor-Expansion Hub-Port: 2
     * armMotor-Expansion Hub-Port: 3
     * intakeOne-Expansion Hub-Port: 0
     * intakeTwo-Expansion Hub-Port: 1
     */
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotorEx sMotor;
    private DcMotor armMotor;
    private DcMotor intakeOne;
    private DcMotor intakeTwo;

    /*
     * Servos:
     * servoOne-Control Hub-Port: 0
     * servoTwo-Control Hub-Port: 1
     * servoLoader-Control Hub-Port: 2
     * servoIn-Control Hub-Port: 3
     */
    private Servo servoOne;
    private Servo servoTwo;
    private Servo servoLoader;
    private Servo servoIn;
    private Servo blocker;


    /*
     * Variables for motor power
     */
    private double forwardPower = .85;
    private double rotationPower = .85;
    private double strafePower = 0.8;
    private double armPower = -.50;
    private double intakeOnePower = -.85;
    private double intakeTwoPower = .95;

    @Override
    public void init() {
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        sMotor = hardwareMap.get(DcMotorEx.class, "sMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intakeOne = hardwareMap.get(DcMotor.class, "intakeOne");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwo");
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoTwo = hardwareMap.get(Servo.class, "servoTwo");
        servoLoader = hardwareMap.get(Servo.class, "servoLoader");
        servoIn = hardwareMap.get(Servo.class, "servoIn");
        blocker = hardwareMap.get(Servo.class, "blocker");
        servoLoader.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        sMotor.setPower(0);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
        servoIn.setPosition(0);

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive();
        load();
        logTelemetry();
    }

    public void stop() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        sMotor.setPower(0);
        armMotor.setPower(0);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
        servoIn.setPosition(0);
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    public void drive() {
//Tank-Drive Controls
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

        flMotor.setPower(Range.clip(Math.signum(flPower) * Math.pow(flPower, 2), -1.0, 1.0));
        frMotor.setPower(Range.clip(Math.signum(frPower) * Math.pow(frPower, 2), -1.0, 1.0));
        blMotor.setPower(Range.clip(Math.signum(blPower) * Math.pow(blPower, 2), -1.0, 1.0));
        brMotor.setPower(Range.clip(Math.signum(brPower) * Math.pow(brPower, 2), -1.0, 1.0));

//Shooting Controls
        int max_velocity = 2500;
        if (this.gamepad2.right_bumper) {
            sMotor.setVelocity(0.63 * max_velocity);
        }
        if (this.gamepad2.left_bumper) {
            sMotor.setPower(0);
        }
        if (this.gamepad2.y) {
            sMotor.setVelocity(0.58 * max_velocity);
        }
        if (this.gamepad2.b) {
            sMotor.setPower(0);
        }

//Arm Controls
        armMotor.setPower(gamepad2.right_stick_y * armPower);
        if (this.gamepad2.a) {
            servoOne.setPosition(0);
            servoTwo.setPosition(1);
        }
        if (this.gamepad2.x) {
            servoOne.setPosition(1);
            servoTwo.setPosition(0);
        }

//Intake Controls
        if (this.gamepad1.right_bumper) {
            intakeOne.setDirection(DcMotor.Direction.REVERSE);
            intakeOne.setPower(intakeOnePower);
            intakeTwo.setPower(intakeTwoPower);
        }
        if (this.gamepad1.left_bumper) {
            intakeOne.setPower(0);
            intakeTwo.setPower(0);
        }
        if (this.gamepad1.y) {
            servoIn.setPosition(1);
        }
        if (this.gamepad1.a) {
            servoIn.setPosition(0);
        }

//Blocker Controls
        if (this.gamepad2.dpad_left){
            blocker.setPosition(1);
        }
        if (this.gamepad2.dpad_right) {
            blocker.setPosition(0);
        }
    }

    public void load() {
        if (sMotor.getVelocity() >= 1600) {
            servoLoader.setPosition(1);
        }
        else {
            servoLoader.setPosition(0);
        }
    }
    public void logTelemetry() {
        telemetry.addData("Shooter power:", sMotor.getVelocity());
        telemetry.addData("frontLeft: ", flMotor.getCurrentPosition());
        telemetry.addData("frontReft: ", frMotor.getCurrentPosition());
        telemetry.addData("backLeft: ", blMotor.getCurrentPosition());
        telemetry.addData("backRight: ", brMotor.getCurrentPosition());
        telemetry.addData("y_pos= ", (flMotor.getCurrentPosition()/53.476));
        telemetry.update();
    }
}

