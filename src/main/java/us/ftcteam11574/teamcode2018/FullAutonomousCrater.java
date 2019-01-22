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

        driveMoveToRelativePosition(240, 240, 0.5);

        if (mineralPosition == GoldMineralLocator.MineralPosition.RIGHT
                || mineralPosition == GoldMineralLocator.MineralPosition.LEFT) {
            double reverseAngle = 1.0;
            if (mineralPosition == GoldMineralLocator.MineralPosition.LEFT)
                reverseAngle = -1.0;
            driveMoveToAngle(reverseAngle * 18, 0.5);
            driveMoveToRelativePosition(500, 500, 0.5);
            driveMoveToRelativePosition(-500, -500, 0.5);
        } else {
            driveMoveToRelativePosition(500, 500, 0.5);
            driveMoveToRelativePosition(-500, -500, 0.5);
        }

        driveMoveToAngle(-50, 0.5);
        waitTime(250);
        driveMoveToRelativePosition(1000, 1000, 0.75);
        driveMoveToAngle(getAngle() - 10, 0.5);
        driveMoveToRelativePosition(100, 100, 0.5);
        driveMoveToAngle(getAngle() -10, 0.5);
        driveMoveToRelativePosition(100, 100, 0.5);
        driveMoveToAngle(-130, 0.25);
        waitTime(250);
        //driveToDiagonalDistance(150, 0.75);
        //driveToDistanceUsingSideSensor(70, 0.5);
        driveMoveToRelativePosition(750, 750, 0.75);

        winchMoveToZero();
        driveMoveToRelativePosition(-1500, -1500, 0.5);
          }
}
