package us.ftcteam11574.teamcode2018;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp
@SuppressWarnings({"unused"})
public class TestGoldMineralLocator extends LinearOpMode {
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

    @Override
    public void runOpMode() {
        GoldMineralLocator goldMineralLocator = new GoldMineralLocator();
        GoldMineralPipeline goldMineralPipeline = new GoldMineralPipeline(goldMineralLocator);
        goldMineralPipeline.init(hardwareMap.appContext,
                CameraViewDisplay.getInstance());
        goldMineralPipeline.enable();

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
    }
}
