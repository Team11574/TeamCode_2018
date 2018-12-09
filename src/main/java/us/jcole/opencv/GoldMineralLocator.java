package us.jcole.opencv;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

@SuppressWarnings({"unused", "WeakerAccess"})
public class GoldMineralLocator {
    public static final double MINERAL_CENTER_MIN = 0.3;
    public static final double MINERAL_CENTER_MAX = 0.7;
    public static final int SELECTED_GOLD_MINERAL_MINIMUM_AREA = 2000;
    public static final int CANDIDATE_GOLD_MINERAL_MINIMUM_AREA =
            SELECTED_GOLD_MINERAL_MINIMUM_AREA / 2;
    public static final Scalar MASK = new Scalar(255);
    final private static Scalar WHITE = new Scalar(255, 255, 255);
    final private static Scalar GREEN = new Scalar(0, 255, 0);
    final private static Scalar RED = new Scalar(255, 0, 0);
    public static final Scalar GOLD_MINERAL_HSV_MIN = new Scalar(10, 127, 80);
    public static final Scalar GOLD_MINERAL_HSV_MAX = new Scalar(30, 255, 255);

    private Rect searchArea;
    private double currentGoldMineralContourArea = 0;
    private double lastKnownGoldMineralContourArea = 0;
    private MineralPosition currentGoldMineralPosition = MineralPosition.UNKNOWN;
    private MineralPosition lastKnownGoldMineralPosition = MineralPosition.UNKNOWN;
    private Rect currentGoldMineralBoundingBox;
    private Rect lastKnownGoldMineralBoundingBox;
    private List<Rect> candidateGoldMineralBoundingBoxes;

    private Mat annotatedImage;

    private static void centeredLabel(Mat image, String label, Rect box, int verticalOffset) {
        Size labelSize = Imgproc.getTextSize(label, 1, 1.0, 1, null);
        double offset = verticalOffset <= 0 ?
                box.tl().y + verticalOffset :
                box.br().y + labelSize.height + verticalOffset;
        Imgproc.putText(image, label,
                new Point(box.tl().x + (double) box.width / 2 - labelSize.width / 2, offset),
                1, 1.0, WHITE, 1);

    }

    private MineralPosition derivePosition(Rect mineralBoundingBox) {
        double mineralLocation =
                (double)(mineralBoundingBox.x - searchArea.x + mineralBoundingBox.width/2) /
                        (double) searchArea.width;

        if (mineralLocation < MINERAL_CENTER_MIN)
            return MineralPosition.LEFT;
        else if (mineralLocation > MINERAL_CENTER_MAX)
            return MineralPosition.RIGHT;
        else
            return MineralPosition.CENTER;
    }

    public Rect getSearchArea() {
        return searchArea;
    }

    public void setSearchArea(Rect searchArea) {
        this.searchArea = searchArea;
    }

    public MineralPosition getCurrentGoldMineralPosition() {
        return currentGoldMineralPosition;
    }

    public MineralPosition getLastKnownGoldMineralPosition() {
        return lastKnownGoldMineralPosition;
    }

    public Rect getCurrentGoldMineralBoundingBox() {
        return currentGoldMineralBoundingBox;
    }

    public Rect getLastKnownGoldMineralBoundingBox() {
        return lastKnownGoldMineralBoundingBox;
    }

    public List<Rect> getCandidateGoldMineralBoundingBoxes() {
        return candidateGoldMineralBoundingBoxes;
    }

    public Mat getAnnotatedImage() {
        return annotatedImage;
    }

    private Mat annotateImage(Mat rgbaImage, Mat grayImage, Mat goldMineralMask) {
        // Fill the annotated image with the grayscale version, so that the
        // details of the matched mask are easier to see.
        grayImage.copyTo(annotatedImage);
        Imgproc.cvtColor(annotatedImage, annotatedImage, Imgproc.COLOR_GRAY2RGBA, 4);

        // Fill in the color image pixels of what appears Gold using the mask.
        rgbaImage.copyTo(annotatedImage, goldMineralMask);

        // Draw a green rectangle around the search area, if defined.
        if (searchArea != null) {
            Imgproc.rectangle(annotatedImage, searchArea.tl(), searchArea.br(), GREEN, 1);
        }

        // Draw red rectangles around all candidate Gold Minerals.
        if (candidateGoldMineralBoundingBoxes != null) {
            for (Rect candidateGoldMineralBoundingBox : candidateGoldMineralBoundingBoxes) {
                Imgproc.rectangle(annotatedImage,
                        candidateGoldMineralBoundingBox.tl(), candidateGoldMineralBoundingBox.br(),
                        RED, 1);
            }
        }

        // If a Gold Mineral was found, draw a bold white rectangle around it
        // and label it with some details.
        if (currentGoldMineralBoundingBox != null) {
            Imgproc.rectangle(annotatedImage,
                    currentGoldMineralBoundingBox.tl(), currentGoldMineralBoundingBox.br(),
                    WHITE, 4);

            centeredLabel(annotatedImage,
                    currentGoldMineralPosition.toString(),
                    currentGoldMineralBoundingBox, -5);

            centeredLabel(annotatedImage,
                    String.format(Locale.US, "%d, %d",
                            currentGoldMineralBoundingBox.x + currentGoldMineralBoundingBox.width / 2,
                            currentGoldMineralBoundingBox.y + currentGoldMineralBoundingBox.height / 2),
                    currentGoldMineralBoundingBox, +5);

            centeredLabel(annotatedImage,
                    String.format(Locale.US, "%.0f / %.0f",
                            currentGoldMineralContourArea, currentGoldMineralBoundingBox.area()),
                    currentGoldMineralBoundingBox, +20);
        }

        return annotatedImage;
    }

    public Mat getOriginalImage() {
        return originalImage;
    }

    private Mat originalImage;
    private Mat hsvImage;
    private Mat searchedImage;
    private Mat goldMineralMask;
    public boolean locate(Mat rgbaImage, Mat grayImage) {
        // These could all be done in the constructor, but OpenCV may not be
        // loaded when this class is constructed, and then these would fail.
        if (originalImage == null)
            originalImage = new Mat();
        if (hsvImage == null)
            hsvImage = new Mat();
        if (searchedImage == null)
            searchedImage = new Mat();
        if (goldMineralMask == null)
            goldMineralMask = new Mat();
        if (annotatedImage == null)
            annotatedImage = new Mat();

        // Save a copy of the original image.
        rgbaImage.copyTo(originalImage);

        // Convert the RGBA image to HSV.
        Imgproc.cvtColor(rgbaImage, hsvImage, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(hsvImage, hsvImage, Imgproc.COLOR_RGB2HSV);

        // If a search area is defined, mask everything else out.
        if (searchArea != null) {
            Mat searchAreaMask = Mat.zeros(rgbaImage.size(), CvType.CV_8U);
            Imgproc.rectangle(searchAreaMask, searchArea.tl(), searchArea.br(), MASK, -1);
            hsvImage.copyTo(searchedImage, searchAreaMask);
        } else {
            hsvImage.copyTo(searchedImage);
        }

        // Search for Gold-colored pixels and produce a mask.
        Core.inRange(searchedImage, GOLD_MINERAL_HSV_MIN, GOLD_MINERAL_HSV_MAX, goldMineralMask);

        // Convert the mask to closed contours.
        List<MatOfPoint> goldMineralContours = new ArrayList<>();
        Imgproc.findContours(goldMineralMask, goldMineralContours, new Mat(),
                Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        currentGoldMineralBoundingBox = null;
        currentGoldMineralPosition = MineralPosition.UNKNOWN;

        // If we found some contours (gold/yellow shapes)...
        if (!goldMineralContours.isEmpty()) {
            // Find the largest masked contour by contour area.
            MatOfPoint selectedContour = Collections.max(goldMineralContours, new Comparator<MatOfPoint>() {
                @Override
                public int compare(MatOfPoint o1, MatOfPoint o2) {
                    return Double.compare(
                            Imgproc.contourArea(o1),
                            Imgproc.contourArea(o2));
                }
            });

            // If the area of the largest contour is large enough, save it as the selected
            // contour. (This discards small matches that would generate a lot of noise.)
            Rect selectedContourBoundingBox = Imgproc.boundingRect(selectedContour);
            double selectedContourArea = Imgproc.contourArea(selectedContour);
            if (selectedContourArea >= SELECTED_GOLD_MINERAL_MINIMUM_AREA) {
                currentGoldMineralContourArea = selectedContourArea;
                currentGoldMineralBoundingBox = selectedContourBoundingBox;
                currentGoldMineralPosition = derivePosition(currentGoldMineralBoundingBox);
            }

            // Save the bounding boxes of all considered bounding boxes that
            // were too small to be selected, but not so small as to definitely
            // be noise. Discard all bounding boxes that are overlapping with
            // the selected contour's bounding box.
            candidateGoldMineralBoundingBoxes = new ArrayList<>();
            for (MatOfPoint candidateMineralContour : goldMineralContours) {
                Rect bb = Imgproc.boundingRect(candidateMineralContour);
                if (bb.area() > CANDIDATE_GOLD_MINERAL_MINIMUM_AREA &&
                        !selectedContourBoundingBox.contains(bb.tl()) &&
                        !selectedContourBoundingBox.contains(bb.br()) &&
                        !selectedContourBoundingBox.contains(new Point(bb.tl().x, bb.br().y)) &&
                        !selectedContourBoundingBox.contains(new Point(bb.br().x, bb.tl().y)))
                    candidateGoldMineralBoundingBoxes.add(bb);
            }
        }

        // We got a good Gold Mineral match. Save it as the last known one.
        if (currentGoldMineralPosition != MineralPosition.UNKNOWN) {
            lastKnownGoldMineralContourArea = currentGoldMineralContourArea;
            lastKnownGoldMineralPosition = currentGoldMineralPosition;
            lastKnownGoldMineralBoundingBox = currentGoldMineralBoundingBox;
        }

        // Store the annotated image.
        annotatedImage = annotateImage(originalImage, grayImage, goldMineralMask);

        // Return true if we got a match.
        return currentGoldMineralPosition != MineralPosition.UNKNOWN;
    }

    public enum MineralPosition {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT,
    }
}
