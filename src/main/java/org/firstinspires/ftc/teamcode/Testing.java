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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name = "Testing", group = "Opmode")


public class Testing extends OpMode {
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
    private double intakeOnePower = -.85;
    private double intakeTwoPower = .95;
    private double powerMultiplier = .9;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDriveCancelable drive;
    boolean currentXState;
    boolean previousXState;

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
        //  telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); //public static
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(new Pose2d()); //transfering between opModes lRR
        // drive.setPoseEstimate(new Pose2d(7.28,-34.602,Math.toRadians(177.608)));
        servoLoader.setPosition(0);
        initObjectDetection();
        //   telemetry.addData("Status", "Initialized");
        //
        //  telemetry.update();
    }

    public void start() {
        sMotor.setPower(0);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
        servoIn.setPosition(0);

        //      telemetry.addData("Status", "Started");
        //  telemetry.update();
    }

    @Override
    public void loop() {
         /*
          if (gamepad1.dpad_up && !targetInProgress) {
           targetInProgress = true;

        if (targetInProgress) {
            if (strafeToTarget() == true) {
                targetInProgress = false;
            }

        }
        */
        drive.update();
        tankDrive();
      //  shooter();
      //  intake();
      //  arm();
      //  blocker();
      //  load();
      //  logTelemetry();
    }


    public void stop() {
        sMotor.setPower(0);
        armMotor.setPower(0);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
        servoIn.setPosition(0);
        //      telemetry.addData("Status", "Stopped");
        // telemetry.update();
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
            drive.update();
        }
    }

    public void shooter() {
        int max_velocity = 2500;
        if (this.gamepad2.right_bumper) {
            sMotor.setVelocity(0.61 * max_velocity);
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
    }

    public void load() {
        if (sMotor.getVelocity() >= 1600) {
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
            blocker.setPosition(1);
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

    //Trajac
    public void spline() {

        Trajectory lineUpShoot = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(7.28, -34.60, Math.toRadians(177.6)))
                .build();
        drive.followTrajectoryAsync(lineUpShoot);
    }



    public void logTelemetry() {
        //telemetry.addData("y_dist value", y_dist);
//        telemetry.addData("Shooter power:", sMotor.getVelocity());
//        telemetry.addData("targetInProgress", targetInProgress);
//        telemetry.update();
    }

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public void initObjectDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        allTrackables.addAll(targetsUltimateGoal);
        /**
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         */

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targetsUltimateGoal.activate();
    }

    /*
        public float imageDetect (){
            VuforiaTrackable trackable = allTrackables.get(1);
            targetVisible = false;
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    return translation.get(1) / mmPerInch;

                }
                else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();


            }
            return -1000;
        }
        float y_dist = -2000;
        public boolean strafeToTarget () {
            y_dist = imageDetect();
           if (y_dist != -1000) {
               if (y_dist > 2 ) {
                   flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                   blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                   frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                   brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                   blMotor.setPower(0.4);
                   brMotor.setPower(0.4);
                   flMotor.setPower(0.4);
                   frMotor.setPower(0.4);
                   return false;
               }
               else if (y_dist < -2) {

                   flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                   blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                   frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                   brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                   blMotor.setPower(0.4);
                   brMotor.setPower(0.4);
                   flMotor.setPower(0.4);
                   frMotor.setPower(0.4);
                   return false;
               }


               blMotor.setPower(0);
               brMotor.setPower(0);
               flMotor.setPower(0);
               frMotor.setPower(0);
               return true;
           }
            blMotor.setPower(0);
            brMotor.setPower(0);
            flMotor.setPower(0);
            frMotor.setPower(0);
           return true;
        }

     */
    public enum DriveState {NORMAL, AUTOMATIC}
}

