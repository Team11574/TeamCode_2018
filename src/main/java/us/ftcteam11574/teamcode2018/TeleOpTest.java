package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@SuppressWarnings({"unused"})
public class TeleOpTest extends OpMode {
    private DcMotor mL, mR, mW;
    private Servo sH;
    private DigitalChannel mWLd;

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
        sH.setPosition(Constants.LATCH_SERVO_CLOSED);

        mWLd = hardwareMap.digitalChannel.get("mWLd");
    }

    @Override
    public void loop() {
        // drivetrain: tank mode with left and right analog sticks
        mL.setPower(-gamepad1.left_stick_y);
        mR.setPower(-gamepad1.right_stick_y);

        // winch motor: left = up, right = down
        if (gamepad1.left_trigger > 0)
            mW.setPower(gamepad1.left_trigger);
        else if (gamepad1.right_trigger > 0 && mWLd.getState() == false)
            mW.setPower(-gamepad1.right_trigger);
        else
            mW.setPower(0);

        // latch servo: b = open, x = close
        if (gamepad1.b)
            sH.setPosition(Constants.LATCH_SERVO_OPEN);
        else if (gamepad1.x)
            sH.setPosition(Constants.LATCH_SERVO_CLOSED);

        telemetry.addData("sH", sH.getPosition());
        telemetry.addData("mW", mW.getCurrentPosition());
        telemetry.addData("mWLd", mWLd.getState());
        telemetry.update();

    }
}



