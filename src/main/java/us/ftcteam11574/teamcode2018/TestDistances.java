package us.ftcteam11574.teamcode2018;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestDistances extends OpMode {
    private Rev2mDistanceSensor drr;
    private Rev2mDistanceSensor drf;
    private Rev2mDistanceSensor df;
    private Rev2mDistanceSensor dd;

    @Override
    public void init() {
        drr = hardwareMap.get(Rev2mDistanceSensor.class , "drr");
        drf = hardwareMap.get(Rev2mDistanceSensor.class , "drf");
        df =  hardwareMap.get(Rev2mDistanceSensor.class , "df");
        dd = hardwareMap.get(Rev2mDistanceSensor.class , "dd");


    }

    @Override
    public void loop() {
        telemetry.addData("drr" , drr.getDistance(DistanceUnit.MM));
        telemetry.addData("drf", drf.getDistance(DistanceUnit.MM));
        telemetry.addData("df" , df.getDistance(DistanceUnit.MM));
        telemetry.addData("dd" , dd.getDistance(DistanceUnit.MM));
        telemetry.update();



    }
}
