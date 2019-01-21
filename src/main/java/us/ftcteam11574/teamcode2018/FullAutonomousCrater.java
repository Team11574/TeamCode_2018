package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import us.jcole.opencv.GoldMineralLocator;

@SuppressWarnings({"unused"})
@Autonomous
public class FullAutonomousCrater extends GenericAutonomous {
    void robotRun(){
        GoldMineralLocator.MineralPosition mineralPosition =
                goldMineralLocator.getLastKnownGoldMineralPosition();

        robotDetachRobotFromLander();

        driveMoveToRelativePosition(300, 300, 0.5);

        if (mineralPosition == GoldMineralLocator.MineralPosition.RIGHT) {
            driveMoveToAngle(25, 0.5);
        } else if (mineralPosition == GoldMineralLocator.MineralPosition.LEFT) {
            driveMoveToAngle(-25, 0.5);
        }

        driveMoveToRelativePosition(500, 500, 0.5);
        driveMoveToRelativePosition(-400, -400, 0.5);
        driveMoveToAngle(-65, 0.5);
        waitTime(250);
        driveToDiagonalDistance(150, 0.75);
        driveMoveToAngle(getAngle() - 10, 0.5);
        driveToDistanceUsingSideSensor(70, 0.5);
        driveMoveToAngle(getAngle() -10, 0.5);
        driveToDistanceParallelToWall(550, 70, 1.0);
        winchMoveToZero();
        double distanceFromWall = getDistanceFromFront();
        driveDistanceParallelToWallUsingEncoders(-(1700 - distanceFromWall), 70, 1.0);


       /* if (mineralPosition == GoldMineralLocator.MineralPosition.UNKNOWN ||
                mineralPosition == GoldMineralLocator.MineralPosition.CENTER) {
            driveMoveToRelativePosition(1100, 1100,
                    Constants.DRIVE_SPEED_TO_PARK);
        } else if (mineralPosition == GoldMineralLocator.MineralPosition.LEFT ||
                mineralPosition==GoldMineralLocator.MineralPosition.RIGHT) {
            double m = 1.0;
            if (mineralPosition == GoldMineralLocator.MineralPosition.RIGHT)
                m = -1.0;
            driveMoveToRelativePosition(180, 180, Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(m * -170, m * 170, Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(850, 850, Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(m * 250, m * -250, Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(450, 450, Constants.DRIVE_SPEED_TO_PARK);
        }
        */

        // winch down below horizontal to drop the team marker
        winchMoveToZero();

    }
}
