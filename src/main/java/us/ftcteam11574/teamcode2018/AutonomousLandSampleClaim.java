package us.ftcteam11574.teamcode2018;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import us.jcole.opencv.GoldMineralLocator;
import us.jcole.opencvpipeline.GoldMineralPipeline;

@SuppressWarnings({"unused"})
@Autonomous
public class AutonomousLandSampleClaim extends LinearOpMode {
    private DcMotor mL, mR, mW;
    private Servo sH;
    private DigitalChannel mWLd;
    private BNO055IMU imu;

    private Rev2mDistanceSensor drr;
    private Rev2mDistanceSensor drf;
    private Rev2mDistanceSensor df;


    GoldMineralLocator goldMineralLocator;
    GoldMineralPipeline goldMineralPipeline;

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

        drr = hardwareMap.get(Rev2mDistanceSensor.class , "drr");
        drf = hardwareMap.get(Rev2mDistanceSensor.class , "drf");
        df =  hardwareMap.get(Rev2mDistanceSensor.class , "df");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        mL = hardwareMap.dcMotor.get("mL");
        mL.setDirection(DcMotorSimple.Direction.REVERSE);
        mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mR = hardwareMap.dcMotor.get("mR");
        mR.setDirection(DcMotorSimple.Direction.FORWARD);
        mR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mW = hardwareMap.dcMotor.get("mW");
        mW.setDirection(DcMotorSimple.Direction.FORWARD);
        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mW.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    private void winchMoveToZero() {
        mW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (mWLd.getState() == false) {
            mW.setPower(-0.6);
        }
        mW.setPower(0);
        mW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private int winchCalculateEncoderCounts(double mm) {
        return (int) (mm * Constants.WINCH_ENCODER_COUNTS_PER_MM);
    }

    private void winchMoveToRelativePosition(double position_mm, double speed) {
        mW.setTargetPosition(mW.getCurrentPosition() + winchCalculateEncoderCounts(position_mm));
        mW.setPower(0.6);

        while (shouldKeepRunning()) {
            if (position_mm > 0.0 && mW.getCurrentPosition() >= mW.getTargetPosition())
                return;

            if (position_mm < 0.0 && mW.getCurrentPosition() <= mW.getTargetPosition())
                return;

            telemetry.addData("mW Current", mW.getCurrentPosition());
            telemetry.addData("mW Target", mW.getTargetPosition());
            telemetry.update();
        }
        if (isStopRequested())
            robotStopAllMotion();
    }

    private int driveCalculateEncoderCounts(double mm) {
        return (int) (mm * Constants.DRIVE_ENCODER_COUNTS_PER_MM);
    }

    private void driveMoveToRelativePosition(double l_position_mm, double r_position_mm, double power) {
        if (l_position_mm != 0.0) {
            mL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mL.setTargetPosition(mL.getCurrentPosition() + driveCalculateEncoderCounts(l_position_mm));
            mL.setPower(power);
        } else {
            mL.setPower(0.0);
        }

        if (r_position_mm != 0.0) {
            mR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mR.setTargetPosition(mR.getCurrentPosition() + driveCalculateEncoderCounts(r_position_mm));
            mR.setPower(power);
        }
        else {
            mR.setPower(0.0);
        }


        while (shouldKeepRunning()) {
            if (r_position_mm > 0.0 && mR.getCurrentPosition() >= mR.getTargetPosition())
                return;

            if (r_position_mm < 0.0 && mR.getCurrentPosition() <= mR.getTargetPosition())
                return;

            if (l_position_mm > 0.0 && mL.getCurrentPosition() >= mL.getTargetPosition())
                return;

            if (l_position_mm < 0.0 && mL.getCurrentPosition() <= mL.getTargetPosition())
                return;

            telemetry.addData("mR Current", mR.getCurrentPosition());
            telemetry.addData("mR Target", mR.getTargetPosition());
            telemetry.addData("mL Current", mL.getCurrentPosition());
            telemetry.addData("mL Target", mL.getTargetPosition());
            telemetry.update();
        }
        if (isStopRequested())
            robotStopAllMotion();
    }

    private double getAngle() {
        return -imu.getAngularOrientation().firstAngle;
    }

    private void driveMoveToAngle(double angle, double power) {
        if (angle < getAngle()) power = -power;
        mL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mL.setPower(power);
        mR.setPower(-power);

        while (shouldKeepRunning()) {
            final double currentAngle = getAngle();
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
        robotStopAllMotion();
    }

    private double getDistanceFromFront() {
        return df.getDistance(DistanceUnit.MM) - 50.0;
    }
    private double getDistanceFromRightFront() {
        return drf.getDistance(DistanceUnit.MM) - 48.0;
    }
    private double getDistanceFromRightRear() {
        return drr.getDistance(DistanceUnit.MM) - 50.0;
    }
    private double getSkew( double distance) {
        double f=getDistanceFromRightFront();
        double r=getDistanceFromRightRear();
        double d=r-f;
        double s = (f - distance)/ distance / 8.0;
        return Range.clip(d/(Math.min(f,r)/2.0) - s, -1.0, 1.0);
    }

    private void driveToDistance(double distance, double power) {
        mL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mL.setPower(power);
        mR.setPower(power);

        while (shouldKeepRunning()) {
            final double currentDistance = df.getDistance(DistanceUnit.MM);
            if (currentDistance <= distance) {
                break;
            }

            telemetry.addData("Current", currentDistance);
            telemetry.addData("Target", distance);
            telemetry.update();
        }
        robotStopAllMotion();
    }

    private void driveToDistanceUsingSideSensor(double distance, double power) {
        mL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mL.setPower(power);
        mR.setPower(power);

        while (shouldKeepRunning()) {
            final double currentDistance = drf.getDistance(DistanceUnit.MM);
            if (currentDistance <= distance) {
                break;
            }

            telemetry.addData("Current", currentDistance);
            telemetry.addData("Target", distance);
            telemetry.update();
        }
        robotStopAllMotion();
    }

    private void driveToDistanceParallelToWall(double distanceAhead, double distanceFromWall, double power) {
        mL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (shouldKeepRunning()) {
            final double currentDistance = df.getDistance(DistanceUnit.MM);
            final double currentSkew = getSkew(distanceFromWall);
            if (currentDistance <= distanceAhead) {
                break;
            }

            if (currentSkew > 0) {
                mR.setPower(power);
                mL.setPower(power * (1.0 - currentSkew));
            } else if (currentSkew < 0) {
                mR.setPower(power * (1.0 + currentSkew));
                mL.setPower(power);
            } else {
                mL.setPower(power);
                mR.setPower(power);
            }

            telemetry.addData("Current", currentDistance);
            telemetry.addData("Target", distanceAhead);
            telemetry.addData("skew",currentSkew);
            telemetry.addData("wall", getDistanceFromRightFront());
            telemetry.update();
        }
        robotStopAllMotion();
    }

    private void driveDistanceParallelToWallUsingEncoders(double distance, double distanceFromWall, double power) {
        mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mL.setTargetPosition(driveCalculateEncoderCounts(distance));
        mR.setTargetPosition(driveCalculateEncoderCounts(distance));

        // TODO ONLY WORKS IN REVERSE
        while (shouldKeepRunning()) {
            final double currentSkew = getSkew(distanceFromWall);
            if (distance < 0 && mL.getCurrentPosition() <= mL.getTargetPosition())
                break;

            if (distance < 0 && mR.getCurrentPosition() <= mR.getTargetPosition())
                break;

            if (currentSkew > 0) {
                mR.setPower(power * (1.0 - currentSkew));
                mL.setPower(power);
            } else if (currentSkew < 0) {
                mR.setPower(power);
                mL.setPower(power * (1.0 + currentSkew));
            } else {
                mL.setPower(power);
                mR.setPower(power);
            }

            telemetry.addData("skew",currentSkew);
            telemetry.update();
        }
        robotStopAllMotion();
    }

    private void hingeUnlatch(){
        sH.setPosition(Constants.LATCH_SERVO_OPEN);
    }

    private void hingeLatch(){
        sH.setPosition(Constants.LATCH_SERVO_OPEN);
    }

    private void robotRun(){
        // winch up a small amount to release the latch
        winchMoveToZero();

        // winch all the way down (mostly by gravity)
        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //TODO may change with limit switch position
        winchMoveToRelativePosition (425, Constants.WINCH_SPEED_FAST);
        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // back up drive motors a bit to straighten against lander
        driveMoveToRelativePosition(-70, -70, Constants.DRIVE_SPEED_DETACH);

        // unlatch from lander
        hingeUnlatch();

        // winch hinge down a bit to release from lander
        winchMoveToRelativePosition(-100, Constants.WINCH_SPEED_FAST);

        GoldMineralLocator.MineralPosition mineralPosition =
                goldMineralLocator.getLastKnownGoldMineralPosition();

        driveMoveToRelativePosition(300, 300, 0.5);
        driveMoveToAngle(25, 0.5);
        driveMoveToRelativePosition(500, 500, 0.5);
        driveMoveToRelativePosition(-325, -325, 0.5);
        driveMoveToAngle(-75, 0.5);
        //sleep(1000);
        driveToDistance(650, 0.75);
        //driveMoveToRelativePosition(0, 100, 1.0);
        driveMoveToAngle(getAngle() - 10, 0.5);
        driveToDistanceUsingSideSensor(70, 0.5);
        //driveMoveToRelativePosition(0, 100, 1.0);
        driveMoveToAngle(getAngle() - 10, 0.5);
        driveToDistanceParallelToWall(550, 70, 1.0);
        winchMoveToZero();
        driveDistanceParallelToWallUsingEncoders(-1400, 70, -1.0);

        /*
        if (mineralPosition == GoldMineralLocator.MineralPosition.UNKNOWN ||
                mineralPosition == GoldMineralLocator.MineralPosition.CENTER) {
            driveMoveToRelativePosition(1100, 1100,
                    Constants.DRIVE_SPEED_TO_PARK);
        } else if (mineralPosition == GoldMineralLocator.MineralPosition.LEFT ||
                mineralPosition==GoldMineralLocator.MineralPosition.RIGHT) {
            double m = 1.0;
            if (mineralPosition == GoldMineralLocator.MineralPosition.RIGHT)
                m = -1.0;
            driveMoveToRelativePosition(180, 180,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToAngle(m * -25.0, 0.5);
            driveMoveToRelativePosition(900, 900,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToAngle(m * 35.0, 0.5);
            driveMoveToRelativePosition(450, 450,
                    Constants.DRIVE_SPEED_TO_PARK);
        }

        // winch down below horizontal to drop the team marker
        winchMoveToZero();
        */
    }

    @Override
    public void runOpMode() {
        robotInit();

        // Send telemetry for the Gold Mineral position while waiting for start.
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Current Position",
                    goldMineralLocator.getCurrentGoldMineralPosition());
            telemetry.addData("Last Known Position",
                    goldMineralLocator.getLastKnownGoldMineralPosition());
            telemetry.update();
        }

        // Turn off the pipeline and keep whatever the last thing we saw was.
        goldMineralPipeline.disable();

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
                robotStopAllMotion();
                return;
            }

            if (t instanceof RuntimeException) {
                throw (RuntimeException) t;
            }

            throw new RuntimeException(t);
        }
    }
}
