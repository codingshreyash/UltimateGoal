/***************************************************************************************
 *    Team: Tech-Knights
 *    Authors: Shreyash Ranjan
 *    Mentors: Chetan Ranjan, Tejas Priyadarshi
 *    Organization: FIRST Tech Challenge
 *    Version: 1.0.5
 *    Release Date: 12-28-2020
 ***************************************************************************************/
package org.firstinspires.ftc.teamcode;
/*
 * imports
 */
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
@Autonomous(name = "SRJA10", group = "")
public class SRJA10 extends OpMode {
    public static ArrayList<Function<Void, Boolean>> autoSequence = new ArrayList<>();
    public int sequnceCount;
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

    private BNO055IMU imu;
    Orientation threeAngles;
    double lastAngle;
    double globalAngle;
    boolean droveAlready;
    boolean rotatedAlready;
    Orientation lastAngles = new Orientation();

    String imageDetected = "";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AZcrIgv/////AAABmaP0qt2hbUhysK3WEDuYtEhzEo+N3Y5xGqGA8EVyBvrRXzRPwQQtI0cnizK49MGkI9qKkHhPob1A3CW6shK1ahjCB35reqzRgKIcju/S4ebprvElpxMe4qKxU0piRBvKtjvoogVt2YrI35UpPLux6a8pmazDlDidgX8nJfRU4qINGivjlQpKWWNmS4XpVxeDt+FmWHp+9d49PMibjTzPg2UmwIyTHsiVbkBZNpL9qN8QVY3T05rXUQHr1s9y33SzFa59Vt3zueXV0q02Iab9/XxgfJyymaMRDoFIUpOvhOY9mXx1kMr9VzjN25psHo9gr/P6V4SQ22oAFZ82rGzUzTR0MyeWP9KYnTl78IvDMP16";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void init() {
    autoSequence.clear();
        autoSequence.add((Void) -> detectImage());
        sequnceCount = 0;
        flMotor = hardwareMap.dcMotor.get("flMotor");
        frMotor = hardwareMap.dcMotor.get("frMotor");
        blMotor = hardwareMap.dcMotor.get("blMotor");
        brMotor = hardwareMap.dcMotor.get("brMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        sMotor = hardwareMap.get(DcMotorEx.class, "sMotor");
        intakeOne = hardwareMap.get(DcMotor.class, "intakeOne");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwo");
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoTwo = hardwareMap.get(Servo.class, "servoTwo");
        servoLoader = hardwareMap.get(Servo.class, "servoLoader");
        servoIn = hardwareMap.get(Servo.class, "servoIn");
        blocker = hardwareMap.get(Servo.class, "blocker");
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        threeAngles = new Orientation();
        droveAlready = false;
        rotatedAlready = false;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
        servoLoader.setPosition(0);
    }

    @Override
    public void loop() {
        telemetry.addData("imageDetected", imageDetected);
        telemetry.addData("Shooter Velocity", sMotor.getVelocity());

        if (sequnceCount < autoSequence.size()) {
            Function<Void, Boolean> currentFunction = autoSequence.get(sequnceCount); // get the function from autoSequence at index sequenceCount
            boolean currentFunctionState = currentFunction.apply(null); // run the current function and get its state (true or false)
            if (currentFunctionState == true) {  // if the state is true, it means the current function has completed running
                sequnceCount++; // increment sequenceCount to begin running the next function in the next iteration
                resetSensors();
            }
        }
        telemetry.addData("Current Position ", flMotor.getCurrentPosition());

        threeAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        double currentAngle = threeAngles.firstAngle;
        telemetry.addData("Current Angle ", getAngle());
        telemetry.update();

    }

    public void resetSensors() {
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetAngle();
        servoinPlace = false;
        timer.reset();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;

    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    private boolean driveDistance(double distance, double drivePower) {
        // set motors to run forward for 2500 encoder counts.
        double inchesToTicks = 40.584;
        drivePower/=100;
        int ticks = (int) (distance * inchesToTicks);

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        flMotor.setTargetPosition(ticks);
        frMotor.setTargetPosition(ticks);
        blMotor.setTargetPosition(ticks);
        brMotor.setTargetPosition(ticks);

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(flMotor.getCurrentPosition()) < flMotor.getTargetPosition()) {
            flMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
            return false;
        } else {
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
            //droveAlready = true;
            return true;
        }
    }

    private boolean driveDistanceBackwards(double distance, double drivePower) {
        // set motors to run forward for 2500 encoder counts.
        double inchesToTicks = 40.584;
        drivePower /= 100;

        int ticks = (int) (distance * inchesToTicks);

        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flMotor.setTargetPosition(ticks);
        frMotor.setTargetPosition(ticks);
        blMotor.setTargetPosition(ticks);
        brMotor.setTargetPosition(ticks);

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(flMotor.getCurrentPosition()) < flMotor.getTargetPosition()) {
            flMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
            return false;
        } else {
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
            //droveAlready = true;
            return true;
        }
    }


    //HERE STRAFE LEFT
    private boolean strafeLeft(double distance, double drivePower) {
        // set motors to run forward for 2500 encoder counts.
        double inchesToTicks = 40.584;

        int ticks = (int) (distance * inchesToTicks);

        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flMotor.setTargetPosition(ticks);
        frMotor.setTargetPosition(ticks);
        blMotor.setTargetPosition(ticks);
        brMotor.setTargetPosition(ticks);

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(blMotor.getCurrentPosition()) < blMotor.getTargetPosition()) {
            flMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
            return false;
        } else {
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
            //droveAlready = true;
            return true;
        }
    }

    //HERE STRAFE RIGHT
    private boolean strafeRight(double distance, double drivePower) {
        // set motors to run forward for 2500 encoder counts.
        double inchesToTicks = 40.584;
        drivePower/=100;

        int ticks = (int) (distance * inchesToTicks);

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        flMotor.setTargetPosition(ticks);
        frMotor.setTargetPosition(ticks);
        blMotor.setTargetPosition(ticks);
        brMotor.setTargetPosition(ticks);

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(blMotor.getCurrentPosition()) < blMotor.getTargetPosition()) {
            flMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
            return false;
        } else {
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
            //droveAlready = true;
            return true;
        }
    }


    private boolean rotateLeft(double rotateAngle, double rotatePower) {
        rotatePower/=100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        if (Math.abs(getAngle()) < rotateAngle) {
            flMotor.setPower(rotatePower);
            blMotor.setPower(rotatePower);
            frMotor.setPower(rotatePower);
            brMotor.setPower(rotatePower);

            return false;
        } else {
            flMotor.setPower(0);
            blMotor.setPower(0);
            frMotor.setPower(0);
            brMotor.setPower(0);
            //rotatedAlready = true;
            return true;
        }
    }

    private boolean driveIntake(double distance, double drivePower) { //
        // set motors to run forward for 2500 encoder counts.
        drivePower/=100;
        double inchesToTicks = 40.584;

        int ticks = (int) (distance * inchesToTicks);

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        flMotor.setTargetPosition(ticks);
        frMotor.setTargetPosition(ticks);
        blMotor.setTargetPosition(ticks);
        brMotor.setTargetPosition(ticks);
        intakeOne.setTargetPosition(6000);
        intakeTwo.setTargetPosition(9000);

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(flMotor.getCurrentPosition()) < flMotor.getTargetPosition()) {
            flMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
            intakeOne.setPower(.95);
            intakeTwo.setPower(1);
            return false;
        } else {
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
            intakeOne.setPower(0);
            intakeTwo.setPower(0);
            //droveAlready = true;
            return true;
        }
    }


    private boolean driveBackIntake(double distance, double drivePower) { //
        // set motors to run forward for 2500 encoder counts.
        drivePower/=100;
        double inchesToTicks = 40.584;

        int ticks = (int) (distance * inchesToTicks);

        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        flMotor.setTargetPosition(ticks);
        frMotor.setTargetPosition(ticks);
        blMotor.setTargetPosition(ticks);
        brMotor.setTargetPosition(ticks);
        intakeOne.setTargetPosition(6000);
        intakeTwo.setTargetPosition(9000);

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(flMotor.getCurrentPosition()) < flMotor.getTargetPosition()) {
            flMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
            intakeOne.setPower(.95);
            intakeTwo.setPower(1);
            return false;
        } else {
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
            intakeOne.setPower(0);
            intakeTwo.setPower(0);
            //droveAlready = true;
            return true;
        }
    }



    private boolean rotateRight(double rotateAngle, double rotatePower) {
        rotatePower/=100;

        // Don't want to count the encoder while turning
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set all 4 motors in all methods so that there's no carry over from previous method
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (getAngle() > -rotateAngle) {
            telemetry.addData("curragnle", Math.abs(getAngle()));
            flMotor.setPower(rotatePower);
            blMotor.setPower(rotatePower);
            frMotor.setPower(rotatePower);
            brMotor.setPower(rotatePower);

            return false;
        } else {
            flMotor.setPower(0);
            blMotor.setPower(0);
            frMotor.setPower(0);
            brMotor.setPower(0);
            //rotatedAlready = true;
            return true;
        }
    }

    private boolean operateArmUp(double distance, double armDownPower) {
        double inchesToTicks = 53.476;
        int ticks = (int) (distance * inchesToTicks);
        armMotor.setTargetPosition(ticks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(armDownPower);
        if (Math.abs(armMotor.getCurrentPosition()) < armMotor.getTargetPosition()) {
            armMotor.setPower(armDownPower);
            return false;
        } else {
            armMotor.setPower(0);
            return true;
        }

    }

    private boolean operateArmDown(double distance, double armUpPower) {
        double inchesToTicks = 53.476;
        int ticks = (int) (distance * inchesToTicks);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setTargetPosition(ticks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(armUpPower);
        if (Math.abs(armMotor.getCurrentPosition()) < armMotor.getTargetPosition()) {
            armMotor.setPower(armUpPower);
            return false;
        } else {
            armMotor.setPower(0);
            return true;
        }

    }

    private boolean closeArm() {
        servoOne.setPosition(0);
        servoTwo.setPosition(1);
        return true;

    }


    private boolean blockerDown() {
        blocker.setPosition(1);
        return true;
    }

    private boolean blockerUp() {
        blocker.setPosition(0);
        return true;
    }
    private boolean openArm() {
        servoOne.setPosition(1);
        servoTwo.setPosition(0);
        return true;
    }



    ElapsedTime timer = new ElapsedTime();
    boolean servoMoving = false;
    boolean servoinPlace = false;

    private boolean shooterLoader(double shootingPower) {
        sMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sMotor.setPower(shootingPower/100);
        if (Math.abs(sMotor.getVelocity()) < 1500) {
            sMotor.setPower(shootingPower);
            telemetry.addData("Shooter Velocity", sMotor.getVelocity());
            telemetry.update();
            return false;
        }
        return true;
    }

    private boolean load(int position) {

        if (servoLoader.getPosition() != position) {
            servoLoader.setPosition(position);
            return false;
        }
        return true;
    }

    private boolean intakePosition () {
        servoIn.setPosition(0);
        return true;
    }

    private boolean intake (double distance, double intakePower) {

       intakeOne.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        double inchesToTicks = 40.584;
        intakePower/=100;
        int ticks = (int) (distance * inchesToTicks);

        intakeOne.setTargetPosition(ticks);
        intakeTwo.setTargetPosition(ticks);


        intakeOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (Math.abs(intakeTwo.getCurrentPosition()) < intakeTwo.getTargetPosition()) {
            intakeOne.setPower(intakePower);
            intakeTwo.setPower(intakePower);

            return false;
        } else {
            intakeOne.setPower(0);
            intakeTwo.setPower(0);
            return true;
        }
    }

    boolean delayReset = false;
    private boolean delay(double ms) {
        if (!delayReset) {
            timer.reset();
            delayReset = true;
        }
        if(timer.milliseconds() < ms) {
            return false;
        }
        delayReset = false;
        sMotor.setPower(0);
        return true;
    }





    private Boolean detectImage() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getRecognitions();
            int closestConfidence = Integer.MIN_VALUE;
            if (recognitions.size() > 0) {
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
                imageDetected = recognitionLabel;
            } else {
                telemetry.addData("closest object", "Nothing");
                imageDetected = "Nothing";
            }

        }
        //135 = 90*
        if (imageDetected.equals("Quad")) {

            autoSequence.add((Void) -> closeArm());
            autoSequence.add((Void) -> driveDistanceBackwards(6,60));
         //
            autoSequence.add((Void) -> strafeLeft(18, 60));
            autoSequence.add((Void) -> blockerDown());
            autoSequence.add((Void) -> driveDistanceBackwards(43,40));
            autoSequence.add((Void) -> blockerUp());
            autoSequence.add((Void) -> driveDistanceBackwards(54.8,80));

            autoSequence.add((Void) -> rotateRight(180,50));
            autoSequence.add((Void) -> operateArmDown(8,25));
            autoSequence.add((Void) -> openArm());
            autoSequence.add((Void) -> driveDistanceBackwards(30,80));
            autoSequence.add((Void) -> rotateLeft(181,50));
            autoSequence.add((Void) -> strafeRight(14,40));
            autoSequence.add((Void) -> driveDistance(4,60)); //move down
//

            autoSequence.add((Void) -> shooterLoader(70)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(400));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(400));

            autoSequence.add((Void) -> shooterLoader(72)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(400));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(400));

            autoSequence.add((Void) -> shooterLoader(72.5)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(400));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(400));
          //
            autoSequence.add((Void) -> strafeRight(3,40)); //move down
            autoSequence.add((Void) -> intakePosition()); //move down
            autoSequence.add((Void) -> driveIntake(37,30));
            autoSequence.add((Void) -> closeArm());

            autoSequence.add((Void) -> driveBackIntake(32,30));

            autoSequence.add((Void) -> shooterLoader(70)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(400));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(400));

            autoSequence.add((Void) -> shooterLoader(72)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(400));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(400));

            autoSequence.add((Void) -> shooterLoader(72.5)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(400));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(400));


            autoSequence.add((Void) -> driveDistanceBackwards(50,90));
            autoSequence.add((Void) -> rotateLeft(90, 40));
            autoSequence.add((Void) -> driveDistance(8,60));
            autoSequence.add((Void) -> openArm());
            autoSequence.add((Void) -> blockerDown());
            autoSequence.add((Void) -> driveDistanceBackwards(14,80));
            autoSequence.add((Void) -> strafeRight(26,80));

        } else if (imageDetected.equals("Single")) {

            autoSequence.add((Void) -> closeArm());
            autoSequence.add((Void) -> driveDistanceBackwards(27, 25));
            autoSequence.add((Void) -> strafeRight(3, 75));
            autoSequence.add((Void) -> shooterLoader(57)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(300));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(300));

            autoSequence.add((Void) -> shooterLoader(55)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(300));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(300));

            autoSequence.add((Void) -> shooterLoader(55)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(300));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(300));

            autoSequence.add((Void) -> strafeRight(19, 40));
            autoSequence.add((Void) -> driveDistanceBackwards(75, 50));
            autoSequence.add((Void) -> rotateLeft(85, 40));
            autoSequence.add((Void) -> operateArmDown(9, 25));
            autoSequence.add((Void) -> openArm());

            autoSequence.add((Void) -> driveDistanceBackwards(18, 50));
            autoSequence.add((Void) -> strafeRight(97, 50));
            autoSequence.add((Void) -> driveDistance(13, 50));
            autoSequence.add((Void) -> closeArm());
            autoSequence.add((Void) -> delay(300));
            autoSequence.add((Void) -> strafeLeft(97, 50));
            autoSequence.add((Void) -> openArm());
            autoSequence.add((Void) -> delay(300));
            autoSequence.add((Void) -> driveDistanceBackwards(15, 50));
            autoSequence.add((Void) -> strafeRight(30, 50));
        }
        else {
            autoSequence.add((Void) -> closeArm());
            autoSequence.add((Void) -> strafeRight(3, 75));
            autoSequence.add((Void) -> driveDistanceBackwards(27, 25));

           autoSequence.add((Void) -> shooterLoader(73)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(300));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(300));

            autoSequence.add((Void) -> shooterLoader(73)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(300));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(300));

            autoSequence.add((Void) -> shooterLoader(73)); //shoot1
            autoSequence.add((Void) -> load(1));
            autoSequence.add((Void) -> delay(300));
            autoSequence.add((Void) -> load(0)); //pull back
            autoSequence.add((Void) -> delay(300));

            autoSequence.add((Void) -> driveDistanceBackwards(50, 50));
            autoSequence.add((Void) -> rotateLeft(85, 40));
            autoSequence.add((Void) -> operateArmDown(9, 25));
            autoSequence.add((Void) -> openArm());

            autoSequence.add((Void) -> driveDistanceBackwards(16, 50));
            autoSequence.add((Void) -> rotateRight(85, 40));
            autoSequence.add((Void) -> strafeLeft(9, 50));
            autoSequence.add((Void) -> driveDistance(43, 50));
            autoSequence.add((Void) -> closeArm());
            autoSequence.add((Void) -> delay(300));

            autoSequence.add((Void) -> driveDistanceBackwards(43, 50));
            autoSequence.add((Void) -> rotateLeft(85, 40));
            autoSequence.add((Void) -> driveDistance(16, 50));
            autoSequence.add((Void) -> openArm());
        }
        return true;
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
