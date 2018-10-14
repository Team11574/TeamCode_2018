package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class AutonomousTest extends LinearOpMode {
    private DcMotor mL, mR, mW;
    private Servo sH;

    public void robotInit() {
        mL = hardwareMap.dcMotor.get("mL");
        mL.setDirection(DcMotorSimple.Direction.REVERSE);
        mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mR = hardwareMap.dcMotor.get("mR");
        mR.setDirection(DcMotorSimple.Direction.FORWARD);
        mR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mW = hardwareMap.dcMotor.get("mW");
        mW.setDirection(DcMotorSimple.Direction.FORWARD);
        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mW.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sH = hardwareMap.servo.get("sH");
        sH.setDirection(Servo.Direction.REVERSE);
        sH.setPosition(0.0);
    }
    final private static double WINCH_ENCODER_COUNTS_PER_MM = 157.08 / (60 * 2 * 24);

    public int winchCalculateEncoderCounts(double mm) {
        return (int) (mm / WINCH_ENCODER_COUNTS_PER_MM);

    }
    public void winchMoveToPosition(double position_mm){
        mW.setTargetPosition(winchCalculateEncoderCounts(position_mm));
        mW.setPower(0.6);
    }

    public void winchWaitForMove() {
        while(mW.getCurrentPosition() != mW.getTargetPosition()) {
            telemetry.addData("mW Current", mW.getCurrentPosition());
            telemetry.addData("mW Target", mW.getTargetPosition());
            telemetry.update();
        };

    }
    final private static double DRIVE_ENCODER_COUNTS_PER_MM = (20 * 9.6775) / (40 * 1 * 28);

    public int driveCalculateEncoderCounts(double mm) {
        return (int) (mm / DRIVE_ENCODER_COUNTS_PER_MM);

    }
    public void driveMoveToPosition(double position_mm, double power){
        mR.setTargetPosition(driveCalculateEncoderCounts(position_mm));
        mR.setPower(power);
        mL.setTargetPosition(driveCalculateEncoderCounts(position_mm));
        mL.setPower(power);
    }

    public void driveWaitForMove() {
        while(mR.getCurrentPosition() != mR.getTargetPosition()) {
            telemetry.addData("mR Current", mR.getCurrentPosition());
            telemetry.addData("mR Target", mR.getTargetPosition());
            telemetry.update();
        };

    }

    public void hingeUnlatch(){
        sH.setPosition(1.0);
    }
    public void hingeLatch(){
        sH.setPosition(0.0);
    }

    public void robotRun(){
        // winch in a small amount to release the hook
        winchMoveToPosition(-50);
        winchWaitForMove();
        // winch all the way down
        winchMoveToPosition(300);
        winchWaitForMove();
        // back up drive motors a bit
        driveMoveToPosition(-50,0.2);
        driveWaitForMove();
        // unlatch from lander
        hingeUnlatch();
        // winch hinge down a bit
        winchMoveToPosition(125);
        winchWaitForMove();

        driveMoveToPosition(900,0.5);
        driveWaitForMove();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robotInit();
        waitForStart();
        robotRun();

    }
}
