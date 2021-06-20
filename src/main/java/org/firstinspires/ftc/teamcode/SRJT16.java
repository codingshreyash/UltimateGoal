package org.firstinspires.ftc.teamcode; //Package class

/* IMPORT CLASSES NEEDED FOR FTC*/

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/* TELE-OP CLASS*/
@TeleOp(name = "SRJT16", group = "Opmode")
@Disabled
public class SRJT16 extends OpMode {

    // MOTORS

    private DcMotor flMotor; //front left motor
    private DcMotor frMotor; //front right motor
    private DcMotor blMotor; //back left motor
    private DcMotor brMotor; //back right motor
    private DcMotorEx sMotor;  //shooter motor
    private DcMotor armMotor; //arm motor
   //0214 private DcMotor conveyorMotor; //conveyor motor
    private DcMotor intakeOne; //first set of intake
    private DcMotor intakeTwo; //first set of intake to lift off


    //Servos:
    private Servo servoOne;
    private Servo servoTwo;
    private Servo servoLoader;
    int check = 0;

    // VARIABLES FOR DIRECTIONS
    private double forwardPower = 1;
    private double rotationPower = 1;
    private double strafePower = 1;
    private double shooterPower = .8;
    private double armPower = .50;
    //0214   private double conveyorMotorPower = 0.75;
    private double intakeOnePower = .85;
    private double intakeTwoPower = .86;


    @Override
    public void init() {
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        sMotor = hardwareMap.get(DcMotorEx.class, "sMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //0214  conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        intakeOne = hardwareMap.get(DcMotor.class, "intakeOne");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwo");
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoTwo = hardwareMap.get(Servo.class, "servoTwo");
        servoLoader = hardwareMap.get(Servo.class, "servoLoader");

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
        sMotor.setPower(0);
        //0214  conveyorMotor.setPower(0);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);

        // Update Telemetry
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive();
        load();
    }

    public void stop() {
        // Set motor power to 0
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        sMotor.setPower(0);
        armMotor.setPower(0);
        //0214  conveyorMotor.setPower(0);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
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

        //Gamepad2:

        int max_velocity = 2500;

        if (this.gamepad2.right_bumper) {
            sMotor.setVelocity(0.68 * max_velocity);
        }
        if (this.gamepad2.left_bumper) {
            sMotor.setPower(0);
        }


        //sMotor.setPower(gamepad2.right_trigger * shooterPower);

        armMotor.setPower(gamepad2.right_stick_y * armPower);

        if (this.gamepad1.right_bumper) {
            intakeOne.setDirection(DcMotor.Direction.REVERSE);
            //0214  conveyorMotor.setDirection(DcMotor.Direction.REVERSE);
            //0214 conveyorMotor.setPower(conveyorMotorPower);
            intakeOne.setPower(intakeOnePower);
            intakeTwo.setPower(intakeTwoPower);
        }

        if (this.gamepad1.left_bumper) {
            //0214         conveyorMotor.setPower(0);
            intakeOne.setPower(0);
            intakeTwo.setPower(0);
        }



        if (this.gamepad2.a) {
            servoOne.setPosition(0);
            servoTwo.setPosition(1);


        }
        if (this.gamepad2.x) {
            servoOne.setPosition(1);
            servoTwo.setPosition(0);
        }

//        if (this.gamepad2.dpad_down) {
//
//            servoLoader.setPosition(1);
//        }
//        if (this.gamepad2.dpad_up) {
//            servoLoader.setPosition(0);
//        }




        double flPower = yValue + xValue + rValue;
        double frPower = yValue - xValue - rValue;
        double blPower = yValue + xValue - rValue;
        double brPower = yValue - xValue + rValue;

        telemetry.addData("Y Value", yValue);

        flMotor.setPower(Range.clip(Math.signum(flPower) * Math.pow(flPower, 2), -1.0, 1.0));
        frMotor.setPower(Range.clip(Math.signum(frPower) * Math.pow(frPower, 2), -1.0, 1.0));
        blMotor.setPower(Range.clip(Math.signum(blPower) * Math.pow(blPower, 2), -1.0, 1.0));
        brMotor.setPower(Range.clip(Math.signum(brPower) * Math.pow(brPower, 2), -1.0, 1.0));

    }


    ElapsedTime timer = new ElapsedTime();
    boolean servoMoving = false;


    public void load() {
        if (gamepad2.dpad_up && !servoMoving) {
            servoLoader.setPosition(0);
            servoMoving = true;
            timer.reset();
        }
        if (timer.milliseconds() >= 500 && servoMoving) {
            servoLoader.setPosition(1);
            servoMoving = false;
            timer.reset();
        }
    }
}


