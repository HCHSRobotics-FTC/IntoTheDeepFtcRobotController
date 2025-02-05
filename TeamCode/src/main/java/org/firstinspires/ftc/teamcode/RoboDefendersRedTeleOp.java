package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Timer;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import java.util.Set;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name="(Red) RoboDefenders TeleOp")
public class RoboDefendersRedTeleOp extends OpMode {
    //back hanger is high, front hanger is low
    //3550
    private static final int LIFT_HEIGHT_LOW = 820; // Bottom Basket
    private static final int LIFT_HEIGHT_MED = 2000; // Low Chamber
    private static final int LIFT_HEIGHT_HIGH = 1400; // High Chamber
    private static final int LIFT_HEIGHT_TOP = 3850; // Top Basket

    private static final double INTAKE_PRESET_0 = 0;
    private static final double INTAKE_PRESET_1 = .5;
    private static final double INTAKE_PRESET_2 = .7;
    private static final double INTAKE_PRESET_3 = .85;
    private static final double INTAKE_PRESET_4 = .95;

    private static final double INTAKE_POWER = 1;

    private static final double LIFT_POWER = 1;
    private static final int LIFT_RESET_POSITION = 0;

    private static final double LIFT_GRABBER_OPEN = .55;
    private static final double LIFT_GRABBER_CLOSE = .825;

    private static final double INTAKE_TILT_UPRIGHT = .96;
    private static final double INTAKE_TILT_LEAN = .5;

    private static final double FOUR_BAR_TILT_UPRIGHT = .4;
    private static final double FOUR_BAR_TILT_DOWNRIGHT = .545;

    private static final double INTAKE_LEFT = .5;
    private static final double INTAKE_RIGHT = .5;
    private static final double LEFT_INTAKE_EXTEND = .5;
    private static final double RIGHT_INTAKE_EXTEND = .5;


    private static final int LIFT_HANGER = 1000;
    private static final double LIFT_RAISE_POWER = 1;
    private static final int LIFT_RAISE_POSITION = 1000;

    private static final int BACK_HANGER_UP = 1000;

    private static final int DELAY_1_VALUE = 500;
    private static final int DELAY_2_VALUE = 1000;
    private static final int DELAY_3_VALUE = 2000;

    static final double CLAW_TILT_SCORE = .95;
    static final double CLAW_TILT_NORMAL = .52;

    private static final double HANGER_POWER = 1;

    private ElapsedTime runtime = new ElapsedTime();
    private Blinker control_Hub;
    private Servo intakeLeft, intakeRight;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor intakeMotor;
    private NormalizedColorSensor intakeColorSensor;
    private Servo claw;
    private Servo _clawTilt;
    private DcMotor _liftMotor;
    private DcMotor _liftHanger;
    private DcMotor _backHanger;
    private Servo _intakeTilt;
    private Servo _fourBarTiltLeft;
    private Servo _fourBarTiltRight;
    private Servo _hangerExtenderLeft;
    private Servo _hangerExtenderRight;
    private ElapsedTime _runtime;

    private static double LEFT_HANGER_EXTENDED = .75;
    private static double RIGHT_HANGER_EXTENDED = .75;
    private static double LEFT_HANGER_IN = 0;
    private static double RIGHT_HANGER_IN = 0;

    private DcMotor frontleft;
    private DcMotor frontright;
    private Gyroscope imu;

    double y, x, rx;
    double denominator, frontLeftPower, backRightPower, frontRightPower, backLeftPower;
    int _currentLiftHeight = 0;
    double _currentLiftMotorUp = 0;
    double _currentLiftMotorDown = 0;
    boolean _isLiftGrabbing = true;
    double _frontHangerMotorPower = 0;
    double _backHangerMotorPower = 0;
    boolean _isTransferProcessStarted = false;
    double _currentIntakePosition = 0;
    final double INTAKE_GAIN = .01;
    double _currentIntakePower = 0;
    boolean _isIntakeLeaning = false;
    boolean _isIntakeIntaking = true;
    boolean _isIntakeOn = false;
    boolean _isHangerExtending = false;
    boolean _isLowHanging = false;

    boolean _isTeamRed = true;
    boolean _isTeamBlue = !_isTeamRed;

    float[] hsvValues = new float[3];
    double intakeDistance;
    final double INTAKE_RANGE = 5; //degrees
    final double BLUE = 215;
    final double RED = 2;

    //
    // Toggle variables
    //
    boolean _wasRightBumperPressed = false;
    boolean _wasLeftBumperPressed = false;
    boolean _wasDpadUpPressed = false;
    boolean _wasDpadDownPressed = false;
    boolean _wasDpadLeftPressed = false;
    boolean _wasDpadRightPressed = false;
    boolean _wasAPressed = false;
    boolean _wasXPressed = false;
    boolean _wasYPressed = false;
    boolean _wasRightBumper2Pressed = false;
    boolean _wasLeftBumper2Pressed = false;
    boolean _wasDpadUp2Pressed = false;
    boolean _wasDpadRight2Pressed = false;
    boolean _wasDpadLeft2Pressed = false;
    boolean _wasDpadDown2Pressed = false;
    boolean _wasRightStick2Pressed = false;
    boolean _wasA2Pressed = false;
    boolean _wasY2Pressed = false;
    boolean _wasB2Pressed = false;
    boolean _wasX2Pressed = false;
    boolean _wasLeftStickButton2Pressed = false;
    boolean _wasRightStickButton2Pressed = false;
    boolean _wasLiftMovingUp = false;
    boolean _wasLiftMovingDown = false;
    boolean _isLiftScoring = false;
    boolean _isLiftReset = false;
    boolean _areHangersExtended = false;
    private boolean isHangerButtonPressed = false;
    private String HangerAction = Extend;
    private static final String Extend = "EXTEND";
    private static final String Retracted = "RETRACTED";


    // Temporary counts
    //
    int _transitionCount = 0;
    int _startHangCount = 0;

    @Override
    public void init() {
        frontleft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontright = hardwareMap.get(DcMotor.class, "frontRight");
        backleft = hardwareMap.get(DcMotor.class, "backLeft");
        backright = hardwareMap.get(DcMotor.class, "backRight");

        //
        // Works forward, backwards, and rotate
        //  strafing does NOT work
        //
        frontleft .setDirection(DcMotor.Direction.FORWARD);
        backleft  .setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright .setDirection(DcMotor.Direction.REVERSE);

        intakeLeft = hardwareMap.get(Servo.class, "leftIntakeExtension");
        intakeRight = hardwareMap.get(Servo.class, "rightIntakeExtension");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
        _liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        _liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeLeft.setPosition(0);
        intakeRight.setPosition(0);

        _runtime = new ElapsedTime();

        //Prevent lift from rising on its own
        _liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");
        _clawTilt = hardwareMap.get(Servo.class, "clawTilt");
        claw.setPosition(LIFT_GRABBER_OPEN);
        _clawTilt.setPosition(CLAW_TILT_NORMAL);
        _isLiftGrabbing = false;

        _liftHanger = hardwareMap.get(DcMotor.class, "liftHanger");
        _backHanger = hardwareMap.get(DcMotor.class, "backHanger");
        _liftHanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _liftHanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _liftHanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backHanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backHanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backHanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        _intakeTilt.setPosition(INTAKE_TILT_UPRIGHT);
        // TODO - uncomment these
        _fourBarTiltLeft = hardwareMap.get(Servo.class, "fourBarTiltLeft");
        _fourBarTiltRight = hardwareMap.get(Servo.class, "fourBarTiltRight");
        moveFourBarToPosition(FOUR_BAR_TILT_DOWNRIGHT);
        _hangerExtenderLeft = hardwareMap.get(Servo.class, "leftHangerTilt");
        _hangerExtenderRight = hardwareMap.get(Servo.class, "rightHangerTilt");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        printInitTelemetry();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        x =  getAdjustedX();
        y = -getAdjustedY();
        rx = gamepad1.right_stick_x;

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower  = (y - x + (.5 * rx)) / denominator;
        backLeftPower   = (y + x + (.5 * rx)) / denominator;
        frontRightPower = (y - x - (.5 * rx)) / denominator;
        backRightPower  = (y + x - (.5 * rx)) / denominator;

        frontleft .setPower(frontLeftPower);
        backleft  .setPower(backLeftPower);
        frontright.setPower(frontRightPower);
        backright .setPower(backRightPower);

        //
        // Start lift
        //
        if (gamepad1.right_bumper && _wasRightBumperPressed == false) {
            _wasRightBumperPressed = true;
        }

        if (!gamepad1.right_bumper && _wasRightBumperPressed == true) {
            _wasRightBumperPressed = false;
            startLift();
        }

        //
        // Reset lift
        //
        if (gamepad1.left_bumper && _wasLeftBumperPressed == false) {
            _wasLeftBumperPressed = true;
        }

        if (!gamepad1.left_bumper && _wasLeftBumperPressed == true) {
            _wasLeftBumperPressed = false;
            resetLift();
        }

        //
        // Setting lift height
        //

        if (gamepad1.dpad_up && _wasDpadUpPressed == false) {
            _wasDpadUpPressed = true;
        }

        if (!gamepad1.dpad_up && _wasDpadUpPressed == true) {
            _wasDpadUpPressed = false;
            setLiftHeight(LIFT_HEIGHT_TOP);
        }

        if (gamepad1.dpad_down && _wasDpadDownPressed == false) {
            _wasDpadDownPressed = true;
        }

        if (!gamepad1.dpad_down && _wasDpadDownPressed == true) {
            _wasDpadDownPressed = false;
            setLiftHeight(LIFT_HEIGHT_LOW);
        }

        if (gamepad1.dpad_right && _wasDpadRightPressed == false) {
            _wasDpadRightPressed = true;

        }

        if (!gamepad1.dpad_right && _wasDpadRightPressed == true) {
            _wasDpadRightPressed = false;
            setLiftHeight(LIFT_HEIGHT_MED);
        }

        if (gamepad1.dpad_left && _wasDpadLeftPressed == false) {
            _wasDpadLeftPressed = true;
        }

        if (!gamepad1.dpad_left && _wasDpadLeftPressed == true) {
            _wasDpadLeftPressed = false;
            setLiftHeight(LIFT_HEIGHT_HIGH);

        }

        //
        // Manual lift height up
        //

        setManualLiftHeightUp(gamepad1.right_trigger);
        setManualLiftHeightDown(-gamepad1.left_trigger);

        //
        // toggle lift grab/release
        //

        if (gamepad1.a && _wasAPressed == false) {
            _wasAPressed = true;
            toggleLiftGrabRelease();
        }
        if (!gamepad1.a && _wasAPressed == true) {
            _wasAPressed = false;

        }

        //
        // begin low to high procedure
        //

        if (gamepad1.x && _wasXPressed == false) {
            _wasXPressed = true;
        }
        if (!gamepad1.x && _wasXPressed == true) {
            _wasXPressed = false;
            beginLowToHighProcedure();
        }
        //
        // lift hanger extension
        //
        handleLiftHangerExtension(gamepad2.left_stick_x);
        handleBackHangerExtension(gamepad2.right_stick_x);

        //
        // start intake process
        //

        if (gamepad2.right_bumper && _wasRightBumper2Pressed == false) {
            _wasRightBumper2Pressed = true;
        }
        if (!gamepad2.right_bumper && _wasRightBumper2Pressed == true) {
            _wasRightBumper2Pressed = false;
            beginIntakeProcess();
        }

        if (gamepad2.left_bumper && _wasLeftBumper2Pressed == false){
            _wasLeftBumper2Pressed = true;
        }
        if (!gamepad2.left_bumper && _wasLeftBumper2Pressed == true) {
            _wasLeftBumper2Pressed = false;
            endIntakeProcess();
        }



        //
        // Intake presets
        //

        if (gamepad2.dpad_up && _wasDpadUp2Pressed == false) {
            _wasDpadUp2Pressed = true;
        }

        if (!gamepad2.dpad_up && _wasDpadUp2Pressed == true) {
            _wasDpadUp2Pressed = false;
//            _currentIntakePosition = INTAKE_PRESET_1;

            intakeLeft.setPosition(INTAKE_PRESET_1);
            intakeRight.setPosition(INTAKE_PRESET_1);

        }

        if (gamepad2.dpad_down && _wasDpadDown2Pressed == false) {
            _wasDpadDown2Pressed = true;
        }

        if (!gamepad2.dpad_down && _wasDpadDown2Pressed == true) {
            _wasDpadDown2Pressed = false;
            //_currentIntakePosition = INTAKE_PRESET_3;

            intakeLeft.setPosition(INTAKE_PRESET_3);
            intakeRight.setPosition(INTAKE_PRESET_3);

        }

        if (gamepad2.dpad_right && _wasDpadRight2Pressed == false) {
            _wasDpadRight2Pressed = true;

        }

        if (!gamepad2.dpad_right && _wasDpadRight2Pressed == true) {
            _wasDpadRight2Pressed = false;
            intakeLeft.setPosition(INTAKE_PRESET_2);
            intakeRight.setPosition(INTAKE_PRESET_2);

        }

        if (gamepad2.dpad_left && _wasDpadLeft2Pressed == false) {
            _wasDpadLeft2Pressed = true;
        }

        if (!gamepad2.dpad_left && _wasDpadLeft2Pressed == true) {
            _wasDpadLeft2Pressed = false;
            //_currentIntakePosition = INTAKE_PRESET_4;

            intakeLeft.setPosition(INTAKE_PRESET_4);
            intakeRight.setPosition(INTAKE_PRESET_4);
        }

        if (gamepad2.x && _wasX2Pressed == false) {
            _wasX2Pressed = true;
        }

        if (!gamepad2.x && _wasX2Pressed == true) {
            _wasX2Pressed = false;
            intakeLeft.setPosition(0);
            intakeRight.setPosition(0);
            endIntakeProcess();
        }

        handleIntakeManualExtend(gamepad2.right_trigger);
        handleIntakeManualRetract(gamepad2.left_trigger);

        if (gamepad2.a && _wasA2Pressed == false) {
            _wasA2Pressed = true;
        }

        if (!gamepad2.a && _wasA2Pressed == true) {
            _wasA2Pressed = false;
            //toggleIntakePower ();
            toggleLowHang ();
        }

        if (gamepad1.y && _wasYPressed == false) {
            _wasYPressed = true;
        }

        if (!gamepad1.y && _wasYPressed == true) {
            _wasYPressed = false;
            _isLowHanging = true;
        }

        if (gamepad2.y && _wasY2Pressed == false) {
            _wasY2Pressed = true;
        }

        if (!gamepad2.y && _wasY2Pressed == true) {
            _wasY2Pressed = false;
            //toggleIntakeLean ();
            toggleHighHang();
        }

        if(gamepad2.b && _wasB2Pressed == false) {
            _wasB2Pressed =true;
        }

        if (!gamepad2.b && _wasB2Pressed == true) {
            _wasB2Pressed =false;
            toggleIntakeDirection();
        }

        if (gamepad2.left_stick_button && _wasLeftStickButton2Pressed == false) {
            _wasLeftStickButton2Pressed = true;
        }

        if (!gamepad2.left_stick_button && _wasLeftStickButton2Pressed == true) {
            _wasLeftStickButton2Pressed = false;
            prepareForHang();
        }

        if (gamepad2.right_stick_button && _wasRightStickButton2Pressed == false) {
            _wasRightStickButton2Pressed = true;
        }

        if (!gamepad2.right_stick_button && _wasRightStickButton2Pressed == true) {
            _wasRightStickButton2Pressed = false;
            toggleHangerExtend();
        }




        // TODO -- this should be using servos. It shouldn't use _currentIntakePower
        /*
        _currentIntakePosition += _currentIntakePower * INTAKE_GAIN;
        _currentIntakePosition = Math.min(Math.max(-0.125, _currentIntakePosition), 0.5);

        intakeLeft.setPosition(_currentIntakePosition);
        intakeRight.setPosition(_currentIntakePosition);

        if (!_isIntakeOn) {
            intakeMotor.setPower(0);
        } else {
            handleColorSensor();
            if (_isIntakeIntaking){
                intakeMotor.setPower(INTAKE_POWER);
            } else {
                intakeMotor.setPower(-INTAKE_POWER);
            }
        }
        */

        handleIntakeTilt();

        handleScoring();

        updateTelemetry();
    }

    @Override
    public void stop() {

    }

    private double getAdjustedX(){
        return Math.pow(gamepad1.left_stick_x * 1.1, 3);
    }
    private double getAdjustedY() {
        return Math.pow(gamepad1.left_stick_y, 3);
    }

    void updateTelemetry() {
        telemetry.addData("Team", _isTeamBlue ? "blue" : "red");
        telemetry.addData("Motors", "x (%.2f), y (%.2f), rx (%.2f)", x, y, rx);
        telemetry.addData("Powers", "FL (%.2f), BL (%.2f), FR (%.2f), BR (%.2f)",
                frontLeftPower, backLeftPower, frontRightPower, backRightPower);
        telemetry.addData("Lift", "unknown");
        telemetry.addData("Current Lift Height", _currentLiftHeight);
        telemetry.addData("Current Lift Height in encoder counts", _liftMotor.getCurrentPosition());
        telemetry.addData("Current Lift Motor Up", _currentLiftMotorUp);
        telemetry.addData("Current Lift Motor Down", _currentLiftMotorDown);
        telemetry.addData("Lift State", _isLiftGrabbing ? "Grabbing" : "Releasing");
        telemetry.addData("Count of Transitions", _transitionCount);
        telemetry.addData("front hanger motor power", _frontHangerMotorPower);
        telemetry.addData("back hanger motor power", _backHangerMotorPower);
        telemetry.addData("Transfer process", _isTransferProcessStarted ? "Running" : "Stopped");
        telemetry.addData("intakePosition", _currentIntakePosition);
        telemetry.addData("Intake Power", _currentIntakePower);
        telemetry.addData("Is Intake On?", _isIntakeOn);
        telemetry.addData("Start Hang Count", _startHangCount);
        telemetry.addData("Intake Lean State", _isIntakeLeaning ? "Leaning" : "Upright");
        telemetry.addData("Intake Direction", _isIntakeIntaking ? "Intake" : "Expelling");
        telemetry.addData("Color Sensor", hsvValues[0]);
        telemetry.addData("Intake distance", intakeDistance);
        telemetry.addData("Lift Extended", _areHangersExtended);
        telemetry.update();
    }

    //
    // hardware control methods
    //

    void handleColorSensor() {
        intakeDistance = ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.INCH);

        if(intakeDistance < 3) {
            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            if(_isTeamRed) {
                //check for blue
                if(hsvValues[0] > BLUE - INTAKE_RANGE && hsvValues[0] < BLUE + INTAKE_RANGE) {
                    _isIntakeIntaking = false;
                }
            } else {
                //check for team red
                if(hsvValues[0] > RED - INTAKE_RANGE && hsvValues[0] < RED + INTAKE_RANGE) {
                    _isIntakeIntaking = false;
                }
            }
        }
    }

    void startLift() {
        _runtime.reset();
        _isLiftScoring = true;
        _isLiftReset = false;


    }

    void resetLift() {
        _runtime.reset();
        _isLiftScoring = false;
        _isLiftReset = true;
        //setLiftHeight(LIFT_RESET_POSITION);

        //startLift();
    }

    void setLiftHeight(int newLiftHeight){
        _currentLiftHeight = newLiftHeight;
    }

    void setManualLiftHeightUp(double liftMotorPower){

        if (liftMotorPower > 0) {
            _currentLiftMotorUp = liftMotorPower;
            _wasLiftMovingUp = true;
            _liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _liftMotor.setPower(_currentLiftMotorUp);
        }
        else if (liftMotorPower == 0 && _wasLiftMovingUp) {
            _currentLiftMotorUp = 0;
            _wasLiftMovingUp = false;
            _liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _liftMotor.setPower(_currentLiftMotorUp);
        }
    }

    void setManualLiftHeightDown(double liftMotorPower) {

        if (liftMotorPower < 0) {
            _currentLiftMotorDown = liftMotorPower;
            _wasLiftMovingDown = true;
            _liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _liftMotor.setPower(_currentLiftMotorDown);
        }
        else if (liftMotorPower == 0 && _wasLiftMovingDown) {
            _currentLiftMotorDown = 0;
            _wasLiftMovingDown = false;
            _liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _liftMotor.setPower(_currentLiftMotorDown);
        }



    }

    void toggleLiftGrabRelease() {
        _isLiftGrabbing = ! _isLiftGrabbing;
        if (_isLiftGrabbing){
            claw.setPosition(LIFT_GRABBER_CLOSE);
        } else {
            claw.setPosition(LIFT_GRABBER_OPEN);
        }
    }

    void beginLowToHighProcedure() {
        _transitionCount++;

        // TODO
    }

    void handleLiftHangerExtension(double stickPosition) {
        _frontHangerMotorPower = stickPosition;
        _liftHanger.setPower(_frontHangerMotorPower);
    }

    void handleBackHangerExtension(double stickPosition) {
        _backHangerMotorPower = stickPosition;
        _backHanger.setPower(_backHangerMotorPower);
    }

    void beginIntakeProcess() {
        // _intakeTilt.setPosition(INTAKE_TILT_LEAN);
        _isIntakeLeaning = true;
        intakeMotor.setPower(INTAKE_POWER);
        _currentIntakePower = INTAKE_POWER;
    }

    void endIntakeProcess() {
        _currentIntakePower = 0;
        intakeMotor.setPower(0);
        // _intakeTilt.setPosition(INTAKE_TILT_UPRIGHT);
        _isIntakeLeaning = false;
    }

    void setIntakePreset(int intakePreset) {
        _currentIntakePosition = intakePreset;
    }

    void handleIntakeManualExtend(double power) {
        // TODO -- this should activate the extension servos
        /*
        if (power > 0) {
            _currentIntakePower = power;
        }
        if (power == 0 && _currentIntakePower > 0) {
            _currentIntakePower = 0;
        }

        intakeMotor.setPower(_currentIntakePower);
        */
    }

    void handleIntakeManualRetract(double power) {
        // TODO -- this should activate the extension servos
        /*
        if (power > 0) {
            _currentIntakePower = -power;
        }
        if (power == 0 && _currentIntakePower < 0) {
            _currentIntakePower = 0;
        }

        intakeMotor.setPower(_currentIntakePower);
        */
    }

    void toggleIntakePower () {
        _isIntakeOn = ! _isIntakeOn;
        if (_isIntakeOn){
            //always reset the direction whilst turning on the intake
            _isIntakeIntaking = true;
        }
    }

    void startHangProcedure () {
        _startHangCount ++ ;


    }

    void toggleIntakeLean () {
        _isIntakeLeaning = ! _isIntakeLeaning;
    }

    void handleIntakeTilt () {
        if (_isIntakeLeaning) {
            _intakeTilt.setPosition(INTAKE_TILT_LEAN);
        } else {
            _intakeTilt.setPosition(INTAKE_TILT_UPRIGHT);
        }
    }

    void toggleIntakeDirection () {
        _isIntakeIntaking = ! _isIntakeIntaking;
        intakeMotor.setPower(-1);
    }
    void prepareForHang() {
        //intakeLeft.setPosition(INTAKE_LEFT);
        //intakeRight.setPosition(-INTAKE_RIGHT);

        //_hangerExtenderRight.setPosition(RIGHT_INTAKE_EXTEND);
        //_hangerExtenderLeft.setPosition(-LEFT_INTAKE_EXTEND);

        _liftHanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _liftHanger.setTargetPosition(LIFT_RAISE_POSITION);
        _liftHanger.setPower(LIFT_RAISE_POWER);
    }

    void printInitTelemetry() {
        telemetry.addData("Lift motor", "%d", _liftMotor.getCurrentPosition());
        telemetry.addData("Low Hanger Motor", "%d", _liftHanger.getCurrentPosition());
        telemetry.addData("High Hanger Motor", "%d", _backHanger.getCurrentPosition());
    }

    void handleScoring() {
        if(_isLiftReset){
            resetScoring();
        } else if(_isLiftScoring){
            runScoring();
        }
    }

    void resetScoring() {
        _isLiftScoring = false;

        if (_runtime.milliseconds() < DELAY_1_VALUE) {


            claw.setPosition(LIFT_GRABBER_CLOSE);

            _clawTilt.setPosition(CLAW_TILT_NORMAL);

            moveFourBarToPosition(FOUR_BAR_TILT_DOWNRIGHT);


            _liftMotor.setTargetPosition(LIFT_RESET_POSITION);
            _liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _liftMotor.setPower(LIFT_POWER);
        }else{
            claw.setPosition(LIFT_GRABBER_OPEN);

            _isLiftReset = false;
        }
    }

    void moveFourBarToPosition(double position) {
        _fourBarTiltRight.setPosition(1-position);
        _fourBarTiltLeft.setPosition(position);

        telemetry.addData("Moving 4-Bar to position", "%.2f", position);
    }

    void runScoring() {

        if(_runtime.milliseconds() < DELAY_1_VALUE) {
            claw.setPosition(LIFT_GRABBER_CLOSE);

        } else if(_runtime.milliseconds() < DELAY_2_VALUE) {
            _liftMotor.setTargetPosition(_currentLiftHeight);
            _liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _liftMotor.setPower(LIFT_POWER);

        } else if (_runtime.milliseconds() < DELAY_3_VALUE) {
            moveFourBarToPosition(FOUR_BAR_TILT_UPRIGHT);
            _clawTilt.setPosition(CLAW_TILT_SCORE);

        } else {
            _isLiftScoring = false;
        }


    }
    void toggleLowHang(){
        int currentPosition = _liftHanger.getTargetPosition();
        int newPosition;
        if (currentPosition == LIFT_HANGER){
            newPosition = 0;
        } else {
            newPosition = LIFT_HANGER;
        }
        _liftHanger.setTargetPosition(newPosition);
        _liftHanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _liftHanger.setPower(HANGER_POWER);
    }
    void toggleHighHang(){
        int currentPosition = _backHanger.getTargetPosition();
        int newPosition;
        if (currentPosition == BACK_HANGER_UP){
            newPosition = 0;
        } else {
            newPosition = BACK_HANGER_UP;
        }
        _backHanger.setTargetPosition(newPosition);
        _backHanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backHanger.setPower(HANGER_POWER);
    }
    void toggleHangerExtend(){
        if (_areHangersExtended){
            _areHangersExtended = false;
            _hangerExtenderLeft.setPosition(LEFT_HANGER_IN);
            _hangerExtenderRight.setPosition(RIGHT_HANGER_IN);
        }
        else {
            _areHangersExtended = true;
            _hangerExtenderLeft.setPosition(LEFT_HANGER_EXTENDED);
            _hangerExtenderRight.setPosition(RIGHT_HANGER_EXTENDED);

        }
    }
}
