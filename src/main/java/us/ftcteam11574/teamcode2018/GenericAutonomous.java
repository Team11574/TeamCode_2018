package us.ftcteam11574.teamcode2018;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import us.jcole.opencv.GoldMineralLocator;
import us.jcole.opencvpipeline.GoldMineralPipeline;
import us.jcole.periodicsensor.PeriodicRev2mDistanceSensor;

@SuppressWarnings({"unused"})
abstract public class GenericAutonomous extends LinearOpMode {
    private DcMotor mL, mR, mW;
    private Servo sH;
    private DigitalChannel mWLd;
    private BNO055IMU imu;

    private PeriodicRev2mDistanceSensor drr;
    private PeriodicRev2mDistanceSensor drf;
    private PeriodicRev2mDistanceSensor df;
    private PeriodicRev2mDistanceSensor dd;


    GoldMineralLocator goldMineralLocator;
    GoldMineralPipeline goldMineralPipeline;

    // Tag to log messages to the Android log with.
    private final static String LOG_TAG = "FTC11574";

    public void info(String msg) {
        Log.i(LOG_TAG, msg);
    }

    // An exception to throw to indicate that "Stop" was pressed (or fired
    // automatically due to timer expiration). The robot should stop
    // immediately to avoid penalty points or crashing.
    public class StopImmediatelyException extends RuntimeException {
        public StopImmediatelyException() { super(); }
    }

    // If stop was requested, throw a StopImmediatelyException which will be
    // caught by runOpMode to stop the robot immediately.
    public boolean shouldKeepRunning() {
        if(isStarted() && isStopRequested())
            throw new StopImmediatelyException();
        return true;
    }

    final private File SAVED_IMAGE_PATH =
            Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
    final private SimpleDateFormat SAVED_IMAGE_DATE_FORMAT =
            new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US);

    // Save an OpenCV image Mat (in RGBA format) to disk (in JPEG BGR format)
    // for later analysis.
    private static void saveCapturedImage(File path, String prefix, Mat rgbaImage, String suffix) {
        Mat bgrImage = new Mat();
        Imgproc.cvtColor(rgbaImage, bgrImage, Imgproc.COLOR_RGBA2BGR, 3);
        String filename = prefix + "_" + suffix + ".jpg";
        File file = new File(path, filename);
        Imgcodecs.imwrite(file.toString(), bgrImage);
    }

    private void robotInit() {
        goldMineralLocator = new GoldMineralLocator();
        goldMineralPipeline = new GoldMineralPipeline(goldMineralLocator);
        goldMineralPipeline.init(hardwareMap.appContext,
                CameraViewDisplay.getInstance());
        goldMineralPipeline.enable();

        drr = new PeriodicRev2mDistanceSensor(hardwareMap, "drr", 50);
        df = new PeriodicRev2mDistanceSensor(hardwareMap, "df", 50);
        drf = new PeriodicRev2mDistanceSensor(hardwareMap, "drf", 50);
        dd = new PeriodicRev2mDistanceSensor(hardwareMap, "dd", 50);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        mL = hardwareMap.dcMotor.get("mL");
        mL.setDirection(DcMotorSimple.Direction.REVERSE);
        mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mR = hardwareMap.dcMotor.get("mR");
        mR.setDirection(DcMotorSimple.Direction.FORWARD);
        mR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mW = hardwareMap.dcMotor.get("mW");
        mW.setDirection(DcMotorSimple.Direction.FORWARD);
        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sH = hardwareMap.servo.get("sH");
        sH.setDirection(Servo.Direction.REVERSE);
        sH.setPosition(Constants.LATCH_SERVO_CLOSED);

        mWLd = hardwareMap.digitalChannel.get("mWLd");

    }

    void robotStopAllMotion() {
        mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void winchMoveToZero() {
        info("Winch: moving to zero");
        while (mWLd.getState() == false) {
            mW.setPower(-0.6);
        }
        mW.setPower(0);
        info("Winch: reached zero");
    }

    int winchCalculateEncoderCounts(double mm) {
        return (int) (mm * Constants.WINCH_ENCODER_COUNTS_PER_MM);
    }

    void winchMoveToRelativePosition(double position_mm, double speed) {
        int TargetPosition = mW.getCurrentPosition() + winchCalculateEncoderCounts(position_mm);
        mW.setPower(0.6 * Math.signum(position_mm));

        while (shouldKeepRunning()) {
            int currentPosition = mW.getCurrentPosition();
            if (position_mm > 0.0 && currentPosition >= TargetPosition)
                break;

            if (position_mm < 0.0 && currentPosition <= TargetPosition)
                break;

            telemetry.addData("mW Current", currentPosition);
            telemetry.addData("mW Target", TargetPosition);
            telemetry.update();
        }
        mW.setPower(0.0);
    }

    void winchLowerRobot() {
        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //TODO may change with limit switch position
        winchMoveToRelativePosition (425, Constants.WINCH_SPEED_FAST);
        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitTime(250);

    }

    private int driveCalculateEncoderCounts(double mm) {
        return (int) (mm * Constants.DRIVE_ENCODER_COUNTS_PER_MM);
    }

    void driveMoveToRelativePosition(double l_position_mm, double r_position_mm, double power) {
        int MrTargetPosition = 0;
        int MlTargetPosition = 0;
        if (l_position_mm != 0.0) {
            MlTargetPosition = mL.getCurrentPosition() + driveCalculateEncoderCounts(l_position_mm);
            mL.setPower(power * Math.signum(l_position_mm));
        } else {
            mL.setPower(0.0);
        }

        if (r_position_mm != 0.0) {
            MrTargetPosition = mR.getCurrentPosition() + driveCalculateEncoderCounts(r_position_mm);
            mR.setPower(power * Math.signum(r_position_mm));
        }
        else {
            mR.setPower(0.0);
        }


        while (shouldKeepRunning()) {
            int MrCurrentPosition = mR.getCurrentPosition();
            int MlCurrentPosition = mL.getCurrentPosition();
            if (r_position_mm > 0.0 && MrCurrentPosition >= MrTargetPosition)
                break;

            if (r_position_mm < 0.0 && MrCurrentPosition <= MrTargetPosition)
                break;

            if (l_position_mm > 0.0 && MlCurrentPosition >= MlTargetPosition)
                break;

            if (l_position_mm < 0.0 && MlCurrentPosition <= MlTargetPosition)
                break;

            telemetry.addData("mR Current", MrCurrentPosition);
            telemetry.addData("mR Target", MrTargetPosition);
            telemetry.addData("mL Current", MlCurrentPosition);
            telemetry.addData("mL Target", MlTargetPosition);
            telemetry.update();
        }

        mL.setPower(0.0);
        mR.setPower(0.0);
    }

    double getAngle() {
        return -imu.getAngularOrientation().firstAngle;
    }

    void driveMoveToAngle(double angle, double power) {
        if (angle < getAngle()) power = -power;
        mL.setPower(power);
        mR.setPower(-power);

        while (shouldKeepRunning()) {
            double currentAngle = getAngle();
            if (power > 0 && currentAngle >= angle) {
                break;
            }
            if (power < 0 && currentAngle <= angle) {
                break;
            }

            telemetry.addData("Current", currentAngle);
            telemetry.addData("Target", angle);
            telemetry.update();
        }
        mL.setPower(0.0);
        mR.setPower(0.0);
    }

    double getDistanceFromFront() {
        return df.getValue() - 50.0;    }
    double getDistanceFromRightFront() {
        return drf.getValue() - 48.0;
    }
    double getDistanceFromRightRear() {
        return drr.getValue() - 50.0;
    }
    double getDistanceDiagonal() {
        return dd.getValue() - 135.0;
    }

    double getDistanceFromWall() {
        return (getDistanceFromRightFront() + getDistanceFromRightRear()) / 2.0;
    }

    double getSkew() {
        double f=getDistanceFromRightFront();
        double r=getDistanceFromRightRear();
        double d = r - f;
        return Range.clip(d / 100.0, -1.0, 1.0);
    }

    double getDistanceCorrection(double desiredDistance, double currentDistance) {
        return Range.clip((currentDistance - desiredDistance) / 100.0, -0.1, 0.1);
    }

    void driveToDistance(double distance, double power) {
        info("Driving Straight " + distance);
        mL.setPower(power);
        mR.setPower(power);

        while (shouldKeepRunning()) {
            double currentDistance = getDistanceFromFront();
            if (currentDistance <= distance) {
                info("Reached Distance " + currentDistance
                );
                break;
            }

            telemetry.addData("Current", currentDistance);
            telemetry.addData("Target", distance);
            telemetry.update();
        }
        mL.setPower(0.0);
        mR.setPower(0.0);
    }

    void driveToDiagonalDistance(double distance, double power) {
        info("Driving Straight " + distance);
        mL.setPower(power);
        mR.setPower(power);

        while (shouldKeepRunning()) {
            double currentDistance = getDistanceDiagonal();
            if (currentDistance <= distance) {
                info("Reached Distance " + currentDistance
                );
                break;
            }

            telemetry.addData("Current", currentDistance);
            telemetry.addData("Target", distance);
            telemetry.update();
        }
        mL.setPower(0.0);
        mR.setPower(0.0);
    }


    void driveToDistanceUsingSideSensor(double distance, double power) {
        mL.setPower(power);
        mR.setPower(power);

        while (shouldKeepRunning()) {
            double currentDistance = drf.getValue();
            if (currentDistance <= distance) {
                break;
            }

            telemetry.addData("Current", currentDistance);
            telemetry.addData("Target", distance);
            telemetry.update();
        }
        mL.setPower(0.0);
        mR.setPower(0.0);
    }

    void driveToDistanceParallelToWall(double distanceAhead, double distanceFromWall, double power) {
        info("Drive: parallel to wall");
        //info("Drive: parallel to wall distanceAhead = " + distanceAhead + " distanceFromWall = " + distanceFromWall + " power = " + power);
        //info(String.format(Locale.US, "Drive: parallel to wall distanceAhead = %.2f distanceFromWall = %.2f power = %.2f",
        //        distanceAhead, distanceFromWall, power));

        while (shouldKeepRunning()) {
            double currentDistanceAhead = getDistanceFromFront();

            if (currentDistanceAhead <= distanceAhead) {
                break;
            }

            double currentSkew = getSkew();
            double currentDistanceFromWall =
                    (getDistanceFromRightFront() + getDistanceFromRightRear()) / 2.0;

            double mL_power = power;
            double mR_power = power;


            double currentDistanceCorrection = getDistanceCorrection(distanceFromWall, getDistanceFromRightFront());

            if (currentDistanceCorrection < 0.1) {
                mL_power *= 1.0 - Math.abs(currentDistanceCorrection);
            } else if (currentDistanceCorrection > 0.1) {
                mR_power *= 1.0 - Math.abs(currentDistanceCorrection);
            } else {
                if (currentSkew > 0) {
                    mL_power *= (1.0 - Math.abs(currentSkew));
                } else if (currentSkew < 0) {
                    mR_power *= (1.0 - Math.abs(currentSkew));
                }

            }

            mL.setPower(mL_power);
            mR.setPower(mR_power);

            telemetry.addData("Current", currentDistanceAhead);
            telemetry.addData("Target", distanceAhead);
            telemetry.addData("skew",currentSkew);
            telemetry.addData("wall", currentDistanceFromWall);
            telemetry.update();
        }
        mL.setPower(0.0);
        mR.setPower(0.0);
        info("Drive: stopped at " + df.getValue() + "mm");
    }

    void driveDistanceParallelToWallUsingEncoders(double distance, double distanceFromWall, double power) {
        int MlTargetPosition = mL.getCurrentPosition() + driveCalculateEncoderCounts(distance);
        int MrTargetPosition = mR.getCurrentPosition() + driveCalculateEncoderCounts(distance);
        info("Distance from wall =  " + distanceFromWall);

        // TODO ONLY WORKS IN REVERSE
        while (shouldKeepRunning()) {
            if (distance < 0 && mL.getCurrentPosition() <= MlTargetPosition)
                break;

            if (distance < 0 && mR.getCurrentPosition() <= MrTargetPosition)
                break;

            double currentSkew = getSkew();
            double currentDistanceFromWall =
                    (getDistanceFromRightFront() + getDistanceFromRightRear()) / 2.0;

            double mL_power = power * Math.signum(distance);
            double mR_power = power * Math.signum(distance);


            double currentDistanceCorrection = getDistanceCorrection(distanceFromWall, getDistanceFromRightRear());

            if (currentDistanceCorrection < 0.1) {
                mL_power *= 1.0 - Math.abs(currentDistanceCorrection);
            } else if (currentDistanceCorrection > 0.1) {
                mR_power *= 1.0 - Math.abs(currentDistanceCorrection);
            } else {
                if (currentSkew < 0) {
                    mL_power *= (1.0 - Math.abs(currentSkew));
                } else if (currentSkew > 0) {
                    mR_power *= (1.0 - Math.abs(currentSkew));
                }

            }

            mL.setPower(mL_power);
            mR.setPower(mR_power);

            telemetry.addData("skew",currentSkew);
            telemetry.update();
        }
        mL.setPower(0.0);
        mR.setPower(0.0);
    }

    void hingeUnlatch(){
        sH.setPosition(Constants.LATCH_SERVO_OPEN);
    }

    void hingeLatch(){
        sH.setPosition(Constants.LATCH_SERVO_OPEN);
    }

    void waitTime(double ms) {
        info("Wait started ms = " + ms);
        ElapsedTime elapsedTime = new ElapsedTime();
        while (shouldKeepRunning() && elapsedTime.milliseconds() < ms) {
            Thread.yield();
        }
        info("Wait ended");
    }

    void robotDetachRobotFromLander() {
        info("Step: winch up a small amount to release the latch");
        winchMoveToZero();

        info("Step: winch all the way down (mostly by gravity)");
        winchLowerRobot();

        info("Step: back up drive motors a bit to straighten against lander)");
        driveMoveToRelativePosition(-80, -80, Constants.DRIVE_SPEED_DETACH);

        info("Step: unlatch from lander");
        hingeUnlatch();

        info("Step: winch hinge down a bit to release from lander");
        winchMoveToRelativePosition(-100, Constants.WINCH_SPEED_FAST);
    }

    abstract void robotRun();

    @Override
    public void runOpMode() {
        info("Running Program " + getClass().getName());
        robotInit();

        // Send telemetry for the Gold Mineral position while waiting for start.
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Current Position",
                    goldMineralLocator.getCurrentGoldMineralPosition());
            telemetry.addData("Last Known Position",
                    goldMineralLocator.getLastKnownGoldMineralPosition());
            telemetry.addData("df", df.getValue());
            telemetry.addData("drf", drf.getValue());
            telemetry.addData("drr", drr.getValue());
            telemetry.update();
        }

        // Turn off the pipeline and keep whatever the last thing we saw was.
        goldMineralPipeline.disable();

        if (isStopRequested()) {
            return;
        }

        info("Starting Program " + getClass().getName());

        // Generate a prefix so that all images saved have the same prefix.
        String savedImagePrefix = SAVED_IMAGE_DATE_FORMAT.format(new Date());

        // Save the most recent original image with a suffix of "left",
        // "center", "right", or "unknown" for easier use with the test
        // suite later.
        saveCapturedImage(SAVED_IMAGE_PATH, savedImagePrefix,
                goldMineralLocator.getOriginalImage(),
                goldMineralLocator.getCurrentGoldMineralPosition().
                        toString().toLowerCase());

        // Save the annotated version of the original image.
        saveCapturedImage(SAVED_IMAGE_PATH, savedImagePrefix,
                goldMineralLocator.getAnnotatedImage(),
                "annotated");

        try {
            robotRun();
        } catch (Throwable t) {
            // Expected due to timer expiration or "Stop" button pressed.
            if (t instanceof StopImmediatelyException) {
                info("Stop requested!");
                robotStopAllMotion();
                goldMineralPipeline.disable();
                return;
            }

            // Unexpected exception; log it, and then re-throw a RuntimeException.
            Log.e(LOG_TAG, "Exception caught!", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException) t;
            }

            throw new RuntimeException(t);
        }

        info("Ended Program " + getClass().getName());
    }
}
