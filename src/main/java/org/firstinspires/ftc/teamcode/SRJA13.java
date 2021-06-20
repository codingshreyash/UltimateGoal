package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "SRJA13", group = "")
public class SRJA13 extends LinearOpMode {

    //variables
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
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

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



    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
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
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        threeAngles = new Orientation();
        droveAlready = false;
        rotatedAlready = false;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
        servoLoader.setPosition(0);




        drive = new SampleMecanumDrive(hardwareMap);
        //trajc

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
        drive.setPoseEstimate (new Pose2d(-55.450, -66.574, Math.toRadians(185.383))); //sets as start on line

        Trajectory moveFowardFromLine = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-30.450, -66.574, Math.toRadians(185.383)))
                .build();

        Trajectory dropWobble01 = drive.trajectoryBuilder(moveFowardFromLine.end())
                .lineToLinearHeading(new Pose2d(27.553958342356903, -56.65254461837849, Math.toRadians(273.48337348166086)))
                .build();

        Trajectory getSecondWobble = drive.trajectoryBuilder(dropWobble01.end())
                .lineToLinearHeading(new Pose2d(-29.70177411916484, -50.52361762768403, Math.toRadians(186.33013235154147)))
                .build();

        Trajectory dropWobble02 = drive.trajectoryBuilder(getSecondWobble.end())
                .lineToLinearHeading(new Pose2d(27.553958342356903, -56.65254461837849, Math.toRadians(273.48337348166086)))
                .build();


        Trajectory readyToPowerShot = drive.trajectoryBuilder(dropWobble02.end())
                .lineToLinearHeading(new Pose2d(1.0757877902001807, -9.22009279735589, Math.toRadians(182.20058267709518)))
                .build();







        Trajectory hitPowerShot1 = drive.trajectoryBuilder(readyToPowerShot.end())
                .lineToLinearHeading(new Pose2d (3.8289579176576949, -27.287730999860404, Math.toRadians(183.47382556103395)))
                .build();

        Trajectory hitPowerShot2 = drive.trajectoryBuilder(hitPowerShot1.end())
                .lineToLinearHeading(new Pose2d(2.8073895618576676, -33.0920184737995, Math.toRadians(184.23776855932152)))
                .build();

        Trajectory hitPowerShot3 = drive.trajectoryBuilder(hitPowerShot2.end())
                .lineToLinearHeading(new Pose2d(3.509232979799914, -38.16947603389731, Math.toRadians(182.8371972888754)))
                .build();












        Trajectory moveToEndSpot = drive.trajectoryBuilder(hitPowerShot3.end())
                .lineToLinearHeading(new Pose2d(13.992, -27.252, Math.toRadians(0)))
                .build();

        Trajectory moveToRightSideOfRings = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-33.893558389154883, -87.89757969683169))
                .build();

        Trajectory moveToStraight = drive.trajectoryBuilder(moveToRightSideOfRings.end())
                .lineTo(new Vector2d(-9.893558389154883, -83.89757969683169))
                .build();

        Trajectory shootingTopGoal = drive.trajectoryBuilder(moveToStraight.end())
                .lineTo(new Vector2d(1.561961817739956, -55.25490095350667))
                .build();

        Trajectory rotateFor1Ring = drive.trajectoryBuilder(shootingTopGoal.end())
                .lineToLinearHeading(new Pose2d(17.224, -30.8002009279735589, Math.toRadians(0)))
                .build();

        Trajectory moveToDropWG1 = drive.trajectoryBuilder(rotateFor1Ring.end())
                .lineToLinearHeading(new Pose2d(22.224, -30.8002009279735589, Math.toRadians(0)))
                .build();

        Trajectory moveToStartLine = drive.trajectoryBuilder(moveToDropWG1.end())
                .lineToLinearHeading(new Pose2d(6.561961817739956, -54.25490095350667, Math.toRadians(180)))
                .build();

        Trajectory moveToStarterStack = drive.trajectoryBuilder(moveToStartLine.end())
                .lineTo(new Vector2d(-30.893558389154883, -62.208))
                .build();

        Trajectory readyToPowerShot1 = drive.trajectoryBuilder(moveToStarterStack.end())
                .lineToLinearHeading(new Pose2d(1.0757877902001807, -9.22009279735589, Math.toRadians(182.20058267709518)))
                .build();

        Trajectory getSecondWobble1 = drive.trajectoryBuilder(hitPowerShot3.end())
                .lineToLinearHeading(new Pose2d(-30.00177411916484, -49.52361762768403, Math.toRadians(186.33013235154147)))
                .build();

        Trajectory moveToDropWG12 = drive.trajectoryBuilder(getSecondWobble1.end())
                .lineToLinearHeading(new Pose2d(22.224, -35.8002009279735589, Math.toRadians(0)))
                .build();

        Trajectory moveToEndSpot1 = drive.trajectoryBuilder(moveToDropWG12.end())
                .lineToLinearHeading(new Pose2d(18.224, -50.252, Math.toRadians(0)))
                .build();

        Trajectory moveToStarterStack4 = drive.trajectoryBuilder(shootingTopGoal.end())
                .lineTo(new Vector2d(-30.893558389154883, -61.208))
                .build();

        Trajectory shootingTopGoalAfter4 = drive.trajectoryBuilder(moveToStarterStack4.end())
                .lineTo(new Vector2d(2.561961817739956, -57.25490095350667))
                .build();

        Trajectory moveToStarterStack42 = drive.trajectoryBuilder(shootingTopGoalAfter4.end())
                .lineTo(new Vector2d(-18.893558389154883, -61.208))
                .build();

        Trajectory moveToStarterStack43 = drive.trajectoryBuilder(moveToStarterStack42.end())
                .lineTo(new Vector2d(-44.893558389154883, -61.208))
                .build();

        Trajectory shootingTopGoalAfter421 = drive.trajectoryBuilder(moveToStarterStack43.end())
                .lineTo(new Vector2d(8.061961817739956, -55.25490095350667))
                .build();

        Trajectory dropWobble41 = drive.trajectoryBuilder(shootingTopGoalAfter4.end())
                .lineToLinearHeading(new Pose2d(64.70177411916484, -54.7681762768403, Math.toRadians(320.33013235154147)))
                .build();

        Trajectory getSecondWobble42 = drive.trajectoryBuilder(dropWobble41.end())
                .lineToLinearHeading(new Pose2d(-36.00177411916484, -54.52361762768403, Math.toRadians(186.33013235154147)))
                .build();

        Trajectory dropWobble42 = drive.trajectoryBuilder(getSecondWobble42.end())
                .lineToLinearHeading(new Pose2d(61.70177411916484, -54.7681762768403, Math.toRadians(320.33013235154147)))
                .build();
        Trajectory moveToEndSpot4 = drive.trajectoryBuilder(dropWobble42.end())
                .lineToLinearHeading(new Pose2d(20.224, -50.252, Math.toRadians(0)))
                .build();


        waitForStart(); //Executes action after this:

        if (imageDetected.equals("Quad")) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(pattern);
            telemetry.addData("imageDetected", imageDetected);
            closeArm();
            drive.followTrajectory(moveToRightSideOfRings);
            drive.followTrajectory(moveToStraight);
            sMotor.setPower(0.77);
            drive.followTrajectory(shootingTopGoal);
            loadSequenceSlow();
            loadSequenceSlow();
            loadSequenceSlow();
            intake();
            servoIn.setPosition(0);
            sMotor.setPower(0);
            drive.followTrajectory(moveToStarterStack4);
            sMotor.setPower(0.70);
            drive.followTrajectory(shootingTopGoalAfter4);
            stopIntake();
            loadSequence();
            loadSequence();
            sMotor.setPower(0);
//            intake();
//            drive.followTrajectory(moveToStarterStack42);
//            drive.followTrajectory(moveToStarterStack43);
//            sMotor.setPower(0.73);
//            drive.followTrajectory(shootingTopGoalAfter421);
//            stopIntake();
//            loadSequence();
//            loadSequence();
//            sMotor.setPower(0);
            operateArmDown(10,20);
        //    drive.followTrajectory(shootingTopGoalAfter421);
            drive.followTrajectory(dropWobble41);
            openArm();
            drive.followTrajectory(getSecondWobble42);
            closeArm();
            drive.followTrajectory(dropWobble42);
            openArm();
            drive.followTrajectory(moveToEndSpot4);




        }
        else if (imageDetected.equals("Single")) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
            blinkinLedDriver.setPattern(pattern);
            telemetry.addData("imageDetected", imageDetected);
            closeArm();


            drive.followTrajectory(moveToRightSideOfRings);
            drive.followTrajectory(moveToStraight);
            sMotor.setPower(0.77);
            drive.followTrajectory(shootingTopGoal);
            loadSequence();
            sMotor.setPower(0);
            drive.followTrajectory(rotateFor1Ring);
            operateArmDown(12,.30);
            drive.followTrajectory(moveToDropWG1);
            openArm();
            servoIn.setPosition(0);
            armMotor.setPower(0);
            intake();
            drive.followTrajectory(moveToStartLine);
            drive.followTrajectory(moveToStarterStack);
            shooterLoader(57);
            drive.followTrajectory(readyToPowerShot1);
            stopIntake();
            servoIn.setPosition(1);
            drive.followTrajectory(hitPowerShot1);
            loadSequence();
            drive.followTrajectory(hitPowerShot2);
            loadSequence();
            drive.followTrajectory(hitPowerShot3);
            loadSequence();
            shooterLoader(0); //stop shooter
            drive.followTrajectory(getSecondWobble1);
            closeArm();
            drive.followTrajectory(moveToDropWG12);
            openArm();
            drive.followTrajectory(moveToEndSpot1);



        }
        else {
            closeArm();
            drive.followTrajectory(moveFowardFromLine);
            operateArmDown(12,20);
            drive.followTrajectory(dropWobble01);
            openArm();
            armMotor.setPower(0);
            drive.followTrajectory(getSecondWobble);
            closeArm();
            drive.followTrajectory(dropWobble02);
            openArm();
            shooterLoader(57); //start shooter
            drive.followTrajectory(readyToPowerShot);
            drive.followTrajectory(hitPowerShot1);
            loadSequence();
            drive.followTrajectory(hitPowerShot2);
            loadSequence();
            drive.followTrajectory(hitPowerShot3);
            loadSequence();
            shooterLoader(0); //stop shooter
            drive.followTrajectory(moveToEndSpot);
            servoIn.setPosition(0);
        }
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
    private void shooterLoader(double shootingPower) {
        sMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sMotor.setPower(shootingPower/100);

    }
    private void loadSequence() {
        servoLoader.setPosition(1);
        sleep(300);
        servoLoader.setPosition(0);
        sleep(300);
    }

    private void loadSequenceSlow() {
        servoLoader.setPosition(1);
        sleep(450);
        servoLoader.setPosition(0);
        sleep(450);
    }


    public void intake() {
        intakeOne.setPower(95);
        intakeTwo.setPower(95);
    }

    public void stopIntake() {
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
    }
    private void openArm() {
        servoOne.setPosition(0);
        servoTwo.setPosition(1);
    }
    private void closeArm() {
        servoOne.setPosition(1);
        servoTwo.setPosition(0);
    }
    private void operateArmDown(double distance, double armDownPower) {
        double inchesToTicks = 53.476;
        int ticks = (int) (distance * inchesToTicks);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setTargetPosition(ticks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(armDownPower);
        if (Math.abs(armMotor.getCurrentPosition()) < armMotor.getTargetPosition()) {
            armMotor.setPower(armDownPower);
        } else {
            armMotor.setPower(0);
        }
    }
}
