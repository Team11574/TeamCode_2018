package us.ftcteam11574.teamcode2018;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TestIMU extends OpMode {
    private BNO055IMU imu;
    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        Orientation orientation = imu.getAngularOrientation();
        telemetry.addData("1" , orientation.firstAngle);
        telemetry.addData("2" , orientation.secondAngle);
        telemetry.addData("3" , orientation.thirdAngle);
        telemetry.update();
    }
}
