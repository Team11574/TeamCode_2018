package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestDriveParallelToWall extends GenericAutonomous {
        @Override
    void robotRun() {
       driveToDistanceParallelToWall(200, 70, 1.0);
    }
}
