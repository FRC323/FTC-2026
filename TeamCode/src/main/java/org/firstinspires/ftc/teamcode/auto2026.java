package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Test Auto", group = "Examples")

public class auto2026 extends OpMode {
    final double FEED_TIME_SECONDS = 0.80; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1500; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1450; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1800; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1600; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;
    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private DcMotor intake = null;
    private DcMotor conveyor = null;
    private Servo leftFeeder = null;
    private Servo rightFeeder = null;
    private Servo diverter = null;

    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private auto2026.LaunchState leftLaunchState;
    private auto2026.LaunchState rightLaunchState;

    private enum DiverterDirection {
        LEFT,
        RIGHT
    }
    private auto2026.DiverterDirection diverterDirection = auto2026.DiverterDirection.LEFT;

    private enum IntakeState {
        ON,
        OFF
    }

    private auto2026.IntakeState intakeState = auto2026.IntakeState.OFF;

    private enum LauncherDistance {
        CLOSE,
        FAR
    }

    private auto2026.LauncherDistance launcherDistance = auto2026.LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(21, 123, Math.toRadians(324)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(25, 120, Math.toRadians(324)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose endPose = startPose; //new Pose(40, 35, Math.toRadians(0)); //ending Pose

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, endPoint;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        endPoint = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                if (pathTimer.getElapsedTimeSeconds() > 5){
                    launchLeft(true);
                }

                setPathState(7);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);

                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(endPoint,true);
                    setPathState(8);
                }
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) {
                    leftLaunchState = auto2026.LaunchState.SPIN_UP;
                    leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftFeederTimer.reset();
                }
                break;
            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (leftLauncher.getVelocity() > launcherMin || leftFeederTimer.seconds() > 3) {
                    leftLaunchState = auto2026.LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                //leftFeeder.setPower(FULL_SPEED);
                leftFeederTimer.reset();
                leftFeeder.setPosition(.35);
                leftLaunchState = auto2026.LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftLaunchState = auto2026.LaunchState.IDLE;
                    //leftFeeder.setPower(STOP_SPEED);
                    leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftLauncher.setPower(.6);
                    rightLauncher.setPower(.6);
                    leftFeeder.setPosition(0.15);
                }
                break;
        }
    }

    void launchRight(boolean shotRequested) {
        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    rightLaunchState = auto2026.LaunchState.SPIN_UP;
                    leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFeederTimer.reset();
                }
                break;
            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (leftLauncher.getVelocity() > launcherMin || rightFeederTimer.seconds() > 3) {
                    rightLaunchState = auto2026.LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                //rightFeeder.setPower(FULL_SPEED);
                rightFeederTimer.reset();
                rightFeeder.setPosition(.45);
                rightLaunchState = auto2026.LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rightLaunchState = auto2026.LaunchState.IDLE;
                    //rightFeeder.setPower(STOP_SPEED);
                    leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftLauncher.setPower(.6);
                    rightLauncher.setPower(.6);
                    rightFeeder.setPosition(0.18);
                }
                break;
        }
    }
    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        leftLaunchState = auto2026.LaunchState.IDLE;
        rightLaunchState = auto2026.LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "llauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rlauncher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        leftFeeder = hardwareMap.get(Servo.class, "lfeeder");
        rightFeeder = hardwareMap.get(Servo.class, "rfeeder");
        diverter = hardwareMap.get(Servo.class, "diverter");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
                * MUST make robot go forward. So adjust these two lines based on your first test drive.
                * Note: The settings here assume direct drive on left and right wheels. Gear
                * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        //leftFeeder.setPower(STOP_SPEED);
        //rightFeeder.setPower(STOP_SPEED);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 0, 20));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 0, 20));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        //rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        diverter.setPosition(0);
        leftFeeder.setPosition(.15);
        rightFeeder.setPosition(.15);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", " Drive Train Initialized");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        telemetry.addData("Status", "Initialized");

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

}