package us.ftcteam11574.teamcode2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings({"unused"})
@Autonomous
public class AutonomousTest extends LinearOpMode {
    private DcMotor mL, mR, mW;
    private Servo sH;

    private void robotInit() {
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
        sH.setPosition(Constants.LATCH_SERVO_CLOSED);
    }

    private int winchCalculateEncoderCounts(double mm) {
        return (int) (mm / Constants.WINCH_ENCODER_COUNTS_PER_MM);

    }
    private void winchMoveToPosition(double position_mm){
        mW.setTargetPosition(winchCalculateEncoderCounts(position_mm));
        mW.setPower(0.6);
    }

    private void winchWaitForMove() {
        while(mW.getCurrentPosition() != mW.getTargetPosition()) {
            telemetry.addData("mW Current", mW.getCurrentPosition());
            telemetry.addData("mW Target", mW.getTargetPosition());
            telemetry.update();
        }

    }

    private int driveCalculateEncoderCounts(double mm) {
        return (int) (mm / Constants.DRIVE_ENCODER_COUNTS_PER_MM);

    }

    private void driveMoveToPosition(double position_mm, double power){
        mR.setTargetPosition(driveCalculateEncoderCounts(position_mm));
        mR.setPower(power);
        mL.setTargetPosition(driveCalculateEncoderCounts(position_mm));
        mL.setPower(power);
    }

    private void driveWaitForMove() {
        while(mR.getCurrentPosition() != mR.getTargetPosition()) {
            telemetry.addData("mR Current", mR.getCurrentPosition());
            telemetry.addData("mR Target", mR.getTargetPosition());
            telemetry.update();
        }
    }

    private void hingeUnlatch(){
        sH.setPosition(Constants.LATCH_SERVO_OPEN);
    }

    private void hingeLatch(){
        sH.setPosition(Constants.LATCH_SERVO_OPEN);
    }

    private void robotRun(){
        // winch up a small amount to release the latch
        winchMoveToPosition(-50);
        winchWaitForMove();

        // winch all the way down (mostly by gravity)
        winchMoveToPosition(300);
        winchWaitForMove();

        // back up drive motors a bit to straighten against lander
        driveMoveToPosition(-100, 0.2);
        driveWaitForMove();

        // unlatch from lander
        hingeUnlatch();

        // winch hinge down a bit to release from lander
        winchMoveToPosition(125);
        winchWaitForMove();

        // drive to the crater or depot
        driveMoveToPosition(1000, 0.5);
        driveWaitForMove();

        // winch down below horizontal to drop the team marker
        winchMoveToPosition(-200);
        winchWaitForMove();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robotInit();
        waitForStart();
        robotRun();
    }
}