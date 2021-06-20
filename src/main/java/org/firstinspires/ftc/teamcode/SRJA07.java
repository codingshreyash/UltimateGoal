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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@Autonomous(name = "SRJA07", group = "")
@Disabled
public class SRJA07 extends OpMode {

    //class variable
    public static ArrayList<Function<Void, Boolean>> autoSequence = new ArrayList<>();
    //<> - closure; generics T - default for a generics; closure of ArrayList
    //() - ArrayList constructor
    public int sequnceCount;

    //motors
    private DcMotor flMotor; //front left motor
    private DcMotor frMotor; //front Right motor
    private DcMotor blMotor; //back left motor
    private DcMotor brMotor; //back Right motor
    private DcMotor armMotor; //arm motor
    private DcMotor sMotor;  //shooter motor

    private Servo servoOne;
    private Servo servoTwo;
    private Servo servoLoader;

    //Gyro related
    private BNO055IMU imu; //Gyro I2C port 0, inertial measurement unit
    Orientation threeAngles;
    double lastAngle;
    double globalAngle;
    boolean droveAlready;
    //double rotateAngle = 90;
    boolean rotatedAlready;
    Orientation lastAngles = new Orientation();

    // VISION
    String imageDetected = "";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AZcrIgv/////AAABmaP0qt2hbUhysK3WEDuYtEhzEo+N3Y5xGqGA8EVyBvrRXzRPwQQtI0cnizK49MGkI9qKkHhPob1A3CW6shK1ahjCB35reqzRgKIcju/S4ebprvElpxMe4qKxU0piRBvKtjvoogVt2YrI35UpPLux6a8pmazDlDidgX8nJfRU4qINGivjlQpKWWNmS4XpVxeDt+FmWHp+9d49PMibjTzPg2UmwIyTHsiVbkBZNpL9qN8QVY3T05rXUQHr1s9y33SzFa59Vt3zueXV0q02Iab9/XxgfJyymaMRDoFIUpOvhOY9mXx1kMr9VzjN25psHo9gr/P6V4SQ22oAFZ82rGzUzTR0MyeWP9KYnTl78IvDMP16";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void init() {

        //clear ArrayList called autoSequence
        autoSequence.clear();
        //adding methods to the ArrayList
        autoSequence.add((Void) -> detectImage());
//        autoSequence.add((Void) -> driveDistance(60, 0.5));
//        autoSequence.add((Void) -> rotateLeft(90, 0.5));

        sequnceCount = 0;

        flMotor = hardwareMap.dcMotor.get("flMotor");
        frMotor = hardwareMap.dcMotor.get("frMotor");
        blMotor = hardwareMap.dcMotor.get("blMotor");
        brMotor = hardwareMap.dcMotor.get("brMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        sMotor = hardwareMap.get(DcMotor.class, "sMotor");

        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoTwo = hardwareMap.get(Servo.class, "servoTwo");
        servoLoader = hardwareMap.get(Servo.class, "servoLoader");


        imu = hardwareMap.get(BNO055IMU.class, "imu 1");

        threeAngles = new Orientation();
        droveAlready = false;
        rotatedAlready = false;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        // initialization here.
//        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //reset encoder counts kept by motors.
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

    }

    @Override
    public void loop() {
        telemetry.addData("imageDetected", imageDetected);

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
    public void resetSensors() {
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        resetAngle();
        servoinPlace = false;
//    globalAngle = 0;
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
//        angleReached = false;
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

    private boolean rotateRight(double rotateAngle, double rotatePower) {
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

    private boolean operateArmDown(double distance, double armDownPower) {
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

    private boolean operateArmUp(double distance, double armUpPower) {
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
    private boolean openArm() {
        servoOne.setPosition(1);
        servoTwo.setPosition(0);
        return true;
    }

    private boolean shooter (double distance, double shooterPower) {
        double inchesToTicks = 53.476;
        int ticks = (int) (distance * inchesToTicks);
        sMotor.setTargetPosition(ticks);
        sMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sMotor.setPower(shooterPower);
        if (Math.abs(sMotor.getCurrentPosition()) < sMotor.getTargetPosition()) {
            sMotor.setPower(shooterPower);
            return false;
        } else {
            sMotor.setPower(0);
            return true;
        }

    }

    private boolean load() {
        servoLoader.setPosition(1);
        return true;
    }

    private boolean unLoad() {
        servoLoader.setPosition(0);
        return true;
    }

    ElapsedTime timer = new ElapsedTime();
    boolean servoMoving = false;
    boolean servoinPlace = false;

    private boolean shooterLoader (double distance, double shootingPower) {
        double inchesToTicks = 53.476;
        int ticks = (int) (distance * inchesToTicks);
        sMotor.setTargetPosition(ticks);
        sMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sMotor.setPower(shootingPower);

        if (!servoMoving && !servoinPlace) {
            servoLoader.setPosition(0);
            servoMoving = true;
            timer.reset();
        }
        if (timer.milliseconds() >= 500 && servoMoving) {

            servoLoader.setPosition(1);
            servoMoving = false;
            servoinPlace = true;
            timer.reset();
        }


        if (Math.abs(sMotor.getCurrentPosition()) < sMotor.getTargetPosition()) {
            sMotor.setPower(shootingPower);
            return false;

        } else {
            sMotor.setPower(0);
            return true;
        }

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
        if (imageDetected.equals("Quad")) {
            autoSequence.add((Void) -> closeArm());
            autoSequence.add((Void) -> operateArmDown(7, .50));
            autoSequence.add((Void) -> driveDistance(17, 0.75));
            autoSequence.add((Void) -> rotateRight(158, 0.50));
            autoSequence.add((Void) -> shooter(30, 95)); // down is postive
            autoSequence.add((Void) -> shooterLoader(60, 95));
            autoSequence.add((Void) -> shooterLoader(50, 95));
            autoSequence.add((Void) -> shooterLoader(50, 95));
            autoSequence.add((Void) -> strafeLeft(11, 0.50));
            autoSequence.add((Void) -> rotateLeft(148, 0.75));
            autoSequence.add((Void) -> driveDistance(94, 0.75));
            autoSequence.add((Void) -> strafeRight(13, 0.75));
            autoSequence.add((Void) -> openArm());

            autoSequence.add((Void) -> driveDistanceBackwards(96, 0.75));
            autoSequence.add((Void) -> rotateLeft(74, .50));
            autoSequence.add((Void) -> strafeLeft(7, .75));
            autoSequence.add((Void) -> driveDistance(16, 0.75));
            autoSequence.add((Void) -> closeArm());
            autoSequence.add((Void) -> driveDistanceBackwards(18, 0.75));
            autoSequence.add((Void) -> rotateRight(67, .50));
            autoSequence.add((Void) -> driveDistance(96, 0.75));
            autoSequence.add((Void) -> strafeRight(6, 0.75));
            autoSequence.add((Void) -> openArm());
            autoSequence.add((Void) -> driveDistanceBackwards(22, 0.75));








            // autoSequence.add((Void) -> operateArmUp(7, .50)); // down is postive
        } else if (imageDetected.equals("Single")) {

        } else {





        }
        return true;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
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
