package badnewsbots.hardware;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Hashtable;

// This class serves as a model for the rotating claw on the Power Play robot.
public class RotatingClaw {
    public enum Side {
        FRONT,
        BACK,
    }

    public enum SlideHeight {
        HIGH_GOAL,
        MID_GOAL,
        LOW_GOAL,
        TOP_TWO_CONES,
        THIRD_CONE,
        SECOND_CONE,
        SLIGHTLY_UP,
        BOTTOM,
        OTHER
    }

    private final Hashtable<SlideHeight, Float> slideHeights = new Hashtable<>();

    public enum GripperState {
        GRIPPED,
        RELEASED
    }
    private final Telemetry telemetry;
    private final Servo gripperServo;
    private final Servo wristServo;
    private LimitSwitch zeroLimitSwitch;
    //private final LinearSlide linearSlide1;
    private final LinearSlide linearSlide2;
    private final double frontPosition = 1; // normal is 1
    private final double backPosition = .325; // normal is .4
    private final double grippingPosition = 0.22;
    private final double openPosition = 0.46;
    private final int slideIncrementAmount = 10;
    private boolean motorIsBusy = false;

    private Side currentSide = Side.BACK;
    private GripperState currentGripperState = GripperState.RELEASED;
    private SlideHeight targetSlideHeight = SlideHeight.OTHER;

    public RotatingClaw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.gripperServo = hardwareMap.get(Servo.class, "gripper");
        this.wristServo = hardwareMap.get(Servo.class, "wrist");
        //DcMotorEx slideMotor1 = hardwareMap.get(DcMotorEx.class, "linearSlide1");
        DcMotorEx slideMotor2 = hardwareMap.get(DcMotorEx.class, "linearSlide1");
        // 97.3cm long
        //this.linearSlide1 = new LinearSlide(slideMotor1, 38.30709f, 3251, false);
        this.linearSlide2 = new LinearSlide(slideMotor2, 38.30709f, 3251, true);
        RevTouchSensor touchSensor =  hardwareMap.get(RevTouchSensor.class, "slide2ZeroLimit");
        this.zeroLimitSwitch = new LimitSwitch(touchSensor, telemetry);
        slideHeights.put(SlideHeight.HIGH_GOAL, 33.5f);
        slideHeights.put(SlideHeight.MID_GOAL, 23f);
        slideHeights.put(SlideHeight.LOW_GOAL, 14f);
        slideHeights.put(SlideHeight.TOP_TWO_CONES, 4f);
        slideHeights.put(SlideHeight.THIRD_CONE, 3f);
        slideHeights.put(SlideHeight.SECOND_CONE, 2f);
        slideHeights.put(SlideHeight.SLIGHTLY_UP, 2f);
        slideHeights.put(SlideHeight.BOTTOM, 0f);
    }

    public void update() {
        DcMotorEx motor = linearSlide2.getMotor();
        motorIsBusy = motor.isBusy();
        if (!motorIsBusy && targetSlideHeight == SlideHeight.BOTTOM) {
            motor.setPower(0);
        }
        zeroLimitSwitch.update();
        if (zeroLimitSwitch.wasPressed()) {
            //linearSlide1.zeroAndMoveToAdjLastPosIn();
            linearSlide2.zeroAndMoveToAdjLastPosIn();
        }
    }

    public void gripRaiseAndRotate() {
        int clawCloseWaitTime = 200;
        Thread thread = new Thread(() -> {
            if (!(gripperServo.getPosition() == grippingPosition)) {
                grip();
                try {
                    Thread.sleep(clawCloseWaitTime);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            moveSlidesToPresetHeight(SlideHeight.LOW_GOAL);
            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            rotateToSide(Side.FRONT);
        });
        thread.start();
    }

    public void letGo() {
        Thread thread = new Thread(() -> {
            if (gripperServo.getPosition() == openPosition) {
                grip();
                if (targetSlideHeight == SlideHeight.LOW_GOAL) {
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
            rotateToSide(Side.BACK);
            moveSlidesToPresetHeight(SlideHeight.BOTTOM);
        });
        thread.start();
    }

    public void grip() {
        gripperServo.setPosition(grippingPosition);
        currentGripperState = GripperState.GRIPPED;
    }
    public void release() {
        gripperServo.setPosition(openPosition);
        currentGripperState = GripperState.RELEASED;
    }

    // Grip if ungripped, ungrip if gripped
    public void toggleGrip() {
        if (currentGripperState == GripperState.GRIPPED) {
            release();
        }
        else if (currentGripperState == GripperState.RELEASED) {
            grip();
        }
    }

    public void rotateToSide(Side side) {
        currentSide = side;
        if (side == Side.FRONT) {
            wristServo.setPosition(frontPosition);
        }
        else if (side == Side.BACK) {
            wristServo.setPosition(backPosition);
        }
    }

    public void rotateToOtherSide() {
        if (targetSlideHeight == SlideHeight.BOTTOM) return;
        if (currentSide == Side.FRONT) {
            rotateToSide(Side.BACK);
        }
        else if (currentSide == Side.BACK) {rotateToSide(Side.FRONT);}
    }

    public void moveSlidesToPresetHeight(SlideHeight height) {
        targetSlideHeight = height;
        float heightInches = slideHeights.get(height);
        //linearSlide1.moveToPositionInches(heightInches);
        linearSlide2.moveToPositionInches(heightInches);
    }

    public void incrementSlidePositions() {
        targetSlideHeight = SlideHeight.OTHER;
        //linearSlide1.incrementPositionTicks(slideIncrementAmount);
        linearSlide2.incrementPositionTicks(slideIncrementAmount);
    }

    public void decrementSlidePositions() {
        targetSlideHeight = SlideHeight.OTHER;
        //linearSlide1.decrementPositionTicks(slideIncrementAmount);
        linearSlide2.decrementPositionTicks(slideIncrementAmount);
    }
    public int getSlide2PosTicks() {
        return linearSlide2.getMotor().getCurrentPosition();
    }
    public int getSlide2TargetPosTicks() {return linearSlide2.getMotor().getTargetPosition();}
    public SlideHeight getTargetSlideHeight() {return targetSlideHeight;}
    public GripperState getCurrentGripperState() {return currentGripperState;}
    public Side getCurrentSide() {return currentSide;}
}
