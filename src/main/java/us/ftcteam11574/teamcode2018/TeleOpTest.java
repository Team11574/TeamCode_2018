package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@SuppressWarnings({"unused"})
public class TeleOpTest extends OpMode {
    private DcMotor mL, mR, mW;
    private Servo sH;

    @Override
    public void init() {
        mL = hardwareMap.dcMotor.get("mL");
        mL.setDirection(DcMotorSimple.Direction.REVERSE);
        mR = hardwareMap.dcMotor.get("mR");
        mR.setDirection(DcMotorSimple.Direction.FORWARD);

        mW = hardwareMap.dcMotor.get("mW");
        mW.setDirection(DcMotorSimple.Direction.FORWARD);
        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sH = hardwareMap.servo.get("sH");
        sH.setDirection(Servo.Direction.REVERSE);
        sH.setPosition(0.0);
    }

    @Override
    public void loop() {
        mL.setPower(-gamepad1.left_stick_y);
        mR.setPower(-gamepad1.right_stick_y);
        mW.setPower(gamepad1.right_stick_x);
        sH.setPosition(gamepad1.right_trigger);

        telemetry.addData("mW", mW.getCurrentPosition());
        telemetry.update();

    }
}



