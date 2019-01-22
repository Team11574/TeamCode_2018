package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import us.jcole.opencv.GoldMineralLocator;

@Autonomous
@SuppressWarnings("unused")
public class FullAutonomousDepot extends GenericAutonomous {
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
            // Drive to hit mineral
            driveMoveToRelativePosition(850, 850, 0.5);
            waitTime(250);
            driveMoveToAngle(reverseAngle * -25, 0.5);
            waitTime(250);
            driveMoveToRelativePosition(300, 300, 0.75);
            // winch down below horizontal to drop the team marker
            winchMoveToZero();
            // Return to starting position
            driveMoveToRelativePosition(-300, -300, 0.75);
            driveMoveToAngle(reverseAngle * 18, 0.5);
            driveMoveToRelativePosition(-850, -850, 0.5);


        } else {
            // Drive to hit mineral
            driveMoveToRelativePosition(1000, 1000, 0.5);
            winchMoveToZero();
            // Return to starting position
            driveMoveToRelativePosition(-1000, -1000, 0.5);
        }

        // Turn to face crater
        driveMoveToAngle(-50, 0.5);
        // Wait to catch up
        waitTime(250);
        driveMoveToRelativePosition(1000,1000,0.5);
        // Drive to wall
        //driveToDiagonalDistance(150, 0.75);
        // Straighten out against wall
        driveMoveToAngle(getAngle() - 30, 0.5);
        driveMoveToRelativePosition(500, 500, 0.5);
        // Distance to keep from wall
        //driveToDistanceUsingSideSensor(70, 0.5);
        // Straighten out again
        //driveMoveToAngle(getAngle() -10, 0.5);
        //driveMoveToRelativePosition();
        // Drive into depot
        //driveToDistanceParallelToWall(550, 70, 1.0);
        // Drop off marker
        //winchMoveToZero();
        //double distanceFromWall = getDistanceFromFront();
        // Park in crater
        //driveDistanceParallelToWallUsingEncoders(-(1700 - distanceFromWall), 70, 1.0);


    }
}