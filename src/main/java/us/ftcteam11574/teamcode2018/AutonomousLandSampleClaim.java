package us.ftcteam11574.teamcode2018;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

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

@SuppressWarnings({"unused"})
@Autonomous
public class AutonomousLandSampleClaim extends LinearOpMode {
    private DcMotor mL, mR, mW;
    private Servo sH;
    private DigitalChannel mWLd;

    GoldMineralLocator goldMineralLocator;
    GoldMineralPipeline goldMineralPipeline;

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

        mL = hardwareMap.dcMotor.get("mL");
        mL.setDirection(DcMotorSimple.Direction.REVERSE);
        mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mR = hardwareMap.dcMotor.get("mR");
        mR.setDirection(DcMotorSimple.Direction.FORWARD);
        mR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        return (int) (mm / Constants.WINCH_ENCODER_COUNTS_PER_MM);
    }

    private void winchMoveToRelativePosition(double position_mm, double speed) {
        mW.setTargetPosition(mW.getCurrentPosition() + winchCalculateEncoderCounts(position_mm));
        mW.setPower(0.6);

        while (!isStopRequested()) {
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
        return (int) (mm / Constants.DRIVE_ENCODER_COUNTS_PER_MM);
    }

    private void driveMoveToRelativePosition(double l_position_mm, double r_position_mm, double power) {
        mL.setTargetPosition(mL.getCurrentPosition() + driveCalculateEncoderCounts(l_position_mm));
        mR.setTargetPosition(mR.getCurrentPosition() + driveCalculateEncoderCounts(r_position_mm));
        mL.setPower(power);
        mR.setPower(power);

        while (!isStopRequested()) {
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
        winchMoveToRelativePosition(425, Constants.WINCH_SPEED_FAST);
        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // back up drive motors a bit to straighten against lander
        driveMoveToRelativePosition(-50, -50, Constants.DRIVE_SPEED_DETACH);

        // unlatch from lander
        hingeUnlatch();

        // winch hinge down a bit to release from lander
        winchMoveToRelativePosition(-250, Constants.WINCH_SPEED_FAST);

        GoldMineralLocator.MineralPosition mineralPosition =
                goldMineralLocator.getLastKnownGoldMineralPosition();

        if (mineralPosition == GoldMineralLocator.MineralPosition.UNKNOWN ||
                mineralPosition == GoldMineralLocator.MineralPosition.CENTER) {
            driveMoveToRelativePosition(1100, 1100,
                    Constants.DRIVE_SPEED_TO_PARK);
        } else if (mineralPosition == GoldMineralLocator.MineralPosition.LEFT ||
                mineralPosition==GoldMineralLocator.MineralPosition.RIGHT) {
            double m = 1.0;
            if (mineralPosition == GoldMineralLocator.MineralPosition.RIGHT)
                m = -1.0;
            driveMoveToRelativePosition(90, 90,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(m*-46, m*46,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(450, 450,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(m*110, m*-110,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(225, 225,
                    Constants.DRIVE_SPEED_TO_PARK);
        }

        // winch down below horizontal to drop the team marker
        winchMoveToZero();
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

        robotRun();
    }
}
