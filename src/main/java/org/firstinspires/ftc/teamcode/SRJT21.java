/***************************************************************************************
 *    Team: Tech-Knights
 *    Authors: Shreyash Ranjan
 *    Mentors: Chetan Ranjan, Tejas Priyadarshi
 *    Organization: FIRST Tech Challenge
 *    Version: 20.0.0
 *    Release Date: 04-09-2021
 ***************************************************************************************/
package org.firstinspires.ftc.teamcode;

/*
 * imports
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name = "SRJT21", group = "Opmode")


public class SRJT21 extends OpMode {
    private DriveState driveState = DriveState.NORMAL;
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

    /*
    Sensors
     */
    RevTouchSensor digitalTouch;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    /*
    Image:
     */
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY = "AZcrIgv/////AAABmaP0qt2hbUhysK3WEDuYtEhzEo+N3Y5xGqGA8EVyBvrRXzRPwQQtI0cnizK49MGkI9qKkHhPob1A3CW6shK1ahjCB35reqzRgKIcju/S4ebprvElpxMe4qKxU0piRBvKtjvoogVt2YrI35UpPLux6a8pmazDlDidgX8nJfRU4qINGivjlQpKWWNmS4XpVxeDt+FmWHp+9d49PMibjTzPg2UmwIyTHsiVbkBZNpL9qN8QVY3T05rXUQHr1s9y33SzFa59Vt3zueXV0q02Iab9/XxgfJyymaMRDoFIUpOvhOY9mXx1kMr9VzjN25psHo9gr/P6V4SQ22oAFZ82rGzUzTR0MyeWP9KYnTl78IvDMP16";
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private boolean targetInProgress = false;
    /*
     * Variables for motor power
     */
    private double forwardPower = .85;
    private double rotationPower = .85;
    private double strafePower = 0.8;
    private double armPower = -.50;
    private double intakeOnePower = -.93;
    private double intakeTwoPower = .98;
    private double powerMultiplier = .9;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDriveCancelable drive;
    boolean currentXState;
    boolean previousXState;
    ElapsedTime timer = new ElapsedTime();
    boolean servoMoving = false;

    @Override
    public void init() {
        sMotor = hardwareMap.get(DcMotorEx.class, "sMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intakeOne = hardwareMap.get(DcMotor.class, "intakeOne");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwo");
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoTwo = hardwareMap.get(Servo.class, "servoTwo");
        servoLoader = hardwareMap.get(Servo.class, "servoLoader");
        servoIn = hardwareMap.get(Servo.class, "servoIn");
        blocker = hardwareMap.get(Servo.class, "blocker");
        digitalTouch = hardwareMap.get(RevTouchSensor.class, "digital_touch");
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        lightStrip();
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        servoLoader.setPosition(0);
    }

    public void start() {
        drive.setPoseEstimate (new Pose2d(18.224, -50.252, Math.toRadians(0))); //sets as start on line
        sMotor.setPower(0);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
        servoIn.setPosition(1);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
    }

    @Override
    public void loop() {
        drive.update();
        tankDrive();
        shooter();
        intake();
        arm();
        blocker();
        load();
        loadPS();
        touchWall();
        logTelemetry();
    }

    public void stop() {
        sMotor.setPower(0);
        armMotor.setPower(0);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
        servoIn.setPosition(0);
    }

    public void tankDrive() {
//Tank-Drive Controls
        currentXState = gamepad1.dpad_up;
        if (currentXState && currentXState != previousXState) {
            if (driveState == DriveState.AUTOMATIC) {
                driveState = DriveState.NORMAL;
                drive.cancelFollowing();
            } else if (driveState == DriveState.NORMAL) {
                driveState = DriveState.AUTOMATIC;
                spline();
            }

        }
        previousXState = currentXState;

        if (gamepad1.left_bumper && gamepad1.right_bumper){
            drive.setPoseEstimate (new Pose2d(59.097206372994115, 9.785136553104257, Math.toRadians(1.5194065301640132)));

        }
        if (driveState == DriveState.AUTOMATIC && !drive.isBusy()) {
            driveState = DriveState.NORMAL;
        }
        if (driveState == DriveState.NORMAL) {

            double xValue = gamepad1.right_stick_x * -rotationPower; //turning
            double yValue = gamepad1.left_stick_y * forwardPower; // f and b
            double rValue = (gamepad1.left_trigger - gamepad1.right_trigger) * strafePower;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -yValue * powerMultiplier,
                            -rValue * powerMultiplier,
                            xValue * powerMultiplier
                    )
            );
        }
    }
public void shooter() {
        int max_velocity = 2500;
        if (this.gamepad2.right_bumper) {
            sMotor.setVelocity(0.65 * max_velocity);
        }
        if (this.gamepad2.left_bumper) {
            sMotor.setPower(0);
        }
        if (this.gamepad2.y) {
            sMotor.setVelocity(0.64 * max_velocity);
           //sMotor.setPower(0.77);
        }
        if (this.gamepad2.b) {
            sMotor.setPower(0);
        }
    }

    public void load() {
        if (sMotor.getVelocity() >= 1600 && gamepad2.right_bumper) {
            servoLoader.setPosition(1);
        } else {
            servoLoader.setPosition(0);
        }
    }

    public void loadPS() {
        if (sMotor.getVelocity() >= 1340 && gamepad2.y) {
            servoLoader.setPosition(1);
        } else {
            servoLoader.setPosition(0);
        }
    }

    public void arm() {
        armMotor.setPower(gamepad2.right_stick_y * armPower);
        if (this.gamepad2.a) {
            servoOne.setPosition(0);
            servoTwo.setPosition(1);
        }
        if (this.gamepad2.x) {
            servoOne.setPosition(1);
            servoTwo.setPosition(0);

        }
    }

    public void intake() {
        if (this.gamepad1.right_bumper) {
            intakeOne.setDirection(DcMotor.Direction.REVERSE);
            intakeOne.setPower(intakeOnePower);
            intakeTwo.setPower(intakeTwoPower);
            blocker.setPosition(0);
        }
        if (this.gamepad1.left_bumper) {
            intakeOne.setPower(0);
            intakeTwo.setPower(0);

        }
        if (this.gamepad1.y) {
            servoIn.setPosition(1);
            blocker.setPosition(0);


        }
        if (this.gamepad1.a) {
            servoIn.setPosition(0);
        }
    }

    public void blocker() {
        if (this.gamepad2.dpad_left) {
            blocker.setPosition(1);
        }
        if (this.gamepad2.dpad_right) {
            blocker.setPosition(0);
        }
    }

    public void spline() {
        Trajectory lineUpShoot = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(9.434742121182461, -48.4491794688335, Math.toRadians(179.5554749518657)))
                .build();
        drive.followTrajectoryAsync(lineUpShoot);
    }

    public void touchWall() {
        if (digitalTouch.isPressed() == true) {
            redLED.setState(false);
            greenLED.setState(true);
        } else {
            greenLED.setState(false);
            redLED.setState(true);
        }
    }

    public void lightStrip() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
        blinkinLedDriver.setPattern(pattern);

        if (gamepad1.dpad_up) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(pattern);
        }
    }

    public void logTelemetry() {
        telemetry.addData("Shooter power:", sMotor.getVelocity());
    }
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    public enum DriveState {NORMAL, AUTOMATIC, POWERSHOT}

}

