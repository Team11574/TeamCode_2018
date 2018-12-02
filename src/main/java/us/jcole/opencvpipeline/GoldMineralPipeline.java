package us.jcole.opencvpipeline;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import us.jcole.opencv.GoldMineralLocator;

public class GoldMineralPipeline extends OpenCVPipeline {
    private GoldMineralLocator mGoldMineralLocator;

    public GoldMineralPipeline(GoldMineralLocator goldMineralLocator) {
        mGoldMineralLocator = goldMineralLocator;
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
            Mat rgb = inputFrame.rgba();
        Mat gray = inputFrame.gray();

        if (rgb == null)
            return null;

        if (mGoldMineralLocator.getSearchArea() == null) {
            mGoldMineralLocator.setSearchArea(new Rect(
                    new Point((rgb.width() - 1) * 0.0,
                            (rgb.height() - 1) * 0.1),
                    new Point((rgb.width() - 1) * 1.0,
                            (rgb.height() - 1) * 0.4)));
        }

        mGoldMineralLocator.locate(rgb, gray);

        return mGoldMineralLocator.getAnnotatedImage();
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        return null;
    }
}
