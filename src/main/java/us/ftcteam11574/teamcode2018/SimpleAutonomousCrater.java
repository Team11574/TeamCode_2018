package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import us.jcole.opencv.GoldMineralLocator;

@Autonomous
public class SimpleAutonomousCrater extends GenericAutonomous {
    void robotRun(){

        robotDetachRobotFromLander();
        driveMoveToRelativePosition(240, 240, 0.5);


        GoldMineralLocator.MineralPosition mineralPosition =
                goldMineralLocator.getLastKnownGoldMineralPosition();

        if (mineralPosition == GoldMineralLocator.MineralPosition.RIGHT
                || mineralPosition == GoldMineralLocator.MineralPosition.LEFT) {
            double reverseAngle = 1.0;
            if (mineralPosition == GoldMineralLocator.MineralPosition.LEFT)
                reverseAngle = -1.0;
            driveMoveToAngle(reverseAngle * 18, 0.5);
            driveMoveToRelativePosition(700, 700, 0.5);
        } else {
            driveMoveToRelativePosition(500, 500, 0.5);
        }
        winchMoveToZero();
    }

}
