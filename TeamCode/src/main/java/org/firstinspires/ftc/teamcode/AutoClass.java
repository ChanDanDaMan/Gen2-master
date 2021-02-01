package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoClass extends LinearOpMode {

    public DcMotor fl, fr, bl, br;
    public DcMotor shooter1, shooter2, intake, wobbleArm;
    public Servo kicker;
    public Servo claw;
    //public OpenCvWebcam Webcam1;
    //public EasyOpenCvWebcam.UltimateGoalPipeline pipeline;

    static final double TICKS_PER_REV = 537.6;
    static final double WHEEL_DIAMETER = 100/25.4;
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    OdometryGlobalCoordinateSystem positionUpdate;

    @Override
    public void runOpMode(){

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
            robotPlane(5,0,.5,0,1);
        }

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

    public void ComputeMotorPowers(double vx, double vy, double a, long t){
        final double r = 2;
        final double lx = 12.776;
        final double ly = 14.258;

        double w1 = (1 / r) * (vx - vy - (lx + ly) * a);
        double w2 = (1 / r) * (vx + vy + (lx + ly) * a);
        double w3 = (1 / r) * (vx + vy - (lx + ly) * a);
        double w4 = (1 / r) * (vx - vy + (lx + ly) * a);

        fl.setPower(w1);
        fr.setPower(w2);
        bl.setPower(w3);
        br.setPower(w4);

        sleep(t);
    }

    public void robotPlane(double Xposition, double Yposition, double robotPwr, double robotAngle, double error){
        double X_distance = Xposition - positionUpdate.returnXCoordinate();
        double Y_distance = Yposition - positionUpdate.returnYCoordinate();

        double distance = Math.hypot(X_distance, Y_distance);

        while (opModeIsActive() && distance > error) {
            distance = Math.hypot(X_distance, Y_distance);
            X_distance = Xposition - positionUpdate.returnXCoordinate();
            Y_distance = Yposition - positionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(X_distance, Y_distance));

            double X_component = calculateX(robotMovementAngle, robotPwr);
            double Y_component = calculateY(robotMovementAngle, robotPwr);
            double offCorrection = robotAngle - positionUpdate.returnOrientation();
        }
    }

    private double calculateX(double desiredAngle, double speed){
        return Math.sin(Math.toRadians(desiredAngle) * speed);
    }

    private double calculateY(double desiredAngle, double speed){
        return Math.cos(Math.toRadians(desiredAngle) * speed);
    }
}
