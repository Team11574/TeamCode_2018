package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestDriveBackwardsParallelToWall extends GenericAutonomous {
    @Override
    void robotRun() {
        driveDistanceParallelToWallUsingEncoders(-1500, 70, 0.5);
    }
}
