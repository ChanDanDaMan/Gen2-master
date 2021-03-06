package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TestOpOnly")
public class TestOp extends LinearOpMode {


    public DcMotor fl, fr, bl, br;
    public DcMotor shooter1, shooter2, intake, wobbleArm;
    public Servo kicker;
    public Servo claw;
    //public OpenCvWebcam Webcam1;
    //public EasyOpenCvWebcam.UltimateGoalPipeline pipeline;

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        kicker = hardwareMap.get(Servo.class, "kicker");
        intake = hardwareMap.get(DcMotor.class, "intake");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        claw = hardwareMap.get(Servo.class, "claw");

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        //pipeline = new EasyOpenCvWebcam.UltimateGoalPipeline();
        //Webcam1.setPipeline(pipeline);

        //Webcam1.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        //Webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        //@Override
        //public void onOpened(){
        //Webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        //}
        //});

        //telemetry.addData("Analysis", pipeline.getAnalysis());
        //telemetry.addData("Position", pipeline.position);
        //telemetry.addData("FPS", String.format("%.2f", Webcam1.getFps()));
        //telemetry.update();

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerDrivetrain(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleArm.setTargetPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            drive();

            if (gamepad1.b) {
                shooterPower(1);
            } else {
                shooterPower(0);
            }
            if (gamepad1.a) {
                kicker.setPosition(0);
            } else {
                kicker.setPosition(.5);
            }
            if (gamepad1.x){
                claw.setPosition(1);
            }else if (gamepad1.y){
                claw.setPosition(0);
            }

            if(gamepad1.dpad_down){
                wobbleArm.setTargetPosition(518);
                wobbleArm.setPower(.2);
            }
            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("Wobble Arm", wobbleArm.getCurrentPosition());
            telemetry.update();
        }


    }


    public void drive() {

        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        fl.setPower(-v1);
        fr.setPower(-v2);
        bl.setPower(-v3);
        br.setPower(-v4);

    }


    public void shooterPower(double pwr) {
        shooter1.setPower(pwr);
        shooter2.setPower(pwr);
    }

    public void setZeroPowerDrivetrain(DcMotor.ZeroPowerBehavior behavior) {
        fl.setZeroPowerBehavior(behavior);
        fr.setZeroPowerBehavior(behavior);
        bl.setZeroPowerBehavior(behavior);
        br.setZeroPowerBehavior(behavior);
    }

}