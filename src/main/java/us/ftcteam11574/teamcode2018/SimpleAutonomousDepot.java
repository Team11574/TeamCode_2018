package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import us.jcole.opencv.GoldMineralLocator;

@SuppressWarnings({"unused"})
@Autonomous
public class SimpleAutonomousDepot extends GenericAutonomous {
    void robotRun(){

        robotDetachRobotFromLander();

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
            driveMoveToRelativePosition(180, 180,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(m*-100, m*100,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(900, 900,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(m*220, m*-220,
                    Constants.DRIVE_SPEED_TO_PARK);
            driveMoveToRelativePosition(450, 450,
                    Constants.DRIVE_SPEED_TO_PARK);
        }

        // winch down below horizontal to drop the team marker
        winchMoveToZero();
    }

}