package badnewsbots.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LinearSlide {
    private final DcMotorEx motor;
    private final float LENGTH_IN;
    private final int LENGTH_TICKS;
    private final float MOTOR_TICKS_PER_INCH;
    private final int encoderDirection;
    private int adjustedZeroPosition = 0;
    private int adjustedMaxPosition;
    private float lastPositionInches = 0;
    private int targetPositionTicks;
    private boolean powerCut = false;

    public LinearSlide(DcMotorEx motor, float lengthIn, int lengthTicks, boolean negateTicks) {
        this.motor = motor;
        if (negateTicks) {
            encoderDirection = -1;}
        else {
            encoderDirection = 1;}
        this.LENGTH_IN = lengthIn;
        this.LENGTH_TICKS = lengthTicks;
        this.MOTOR_TICKS_PER_INCH = LENGTH_TICKS / LENGTH_IN;
        zero();
        //motor.setTargetPositionTolerance(0); // test if works once it works normally
    }

    public void zero() {
        adjustedZeroPosition = motor.getCurrentPosition();
        adjustedMaxPosition = encoderDirection * LENGTH_TICKS + adjustedZeroPosition;
    }

    public void zeroAndMoveToAdjLastPosIn() {
        zero();
        moveToPositionInches(lastPositionInches);
    }
    // Tell the slide to move to a given position in ticks
    public void moveToPositionTicks(int positionTicks) {
        if (adjustedZeroPosition > adjustedMaxPosition) {
            positionTicks = Math.min(adjustedZeroPosition, Math.max(positionTicks, adjustedMaxPosition));
        } else {
            positionTicks = Math.max(adjustedZeroPosition, Math.min(positionTicks, adjustedMaxPosition));
        }
        targetPositionTicks = positionTicks;
        motor.setTargetPosition(positionTicks);
        motor.setPower(1);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void incrementPositionTicks(int amount) {
        int newPosition = targetPositionTicks + amount * encoderDirection;
        moveToPositionTicks(Math.round(newPosition + adjustedZeroPosition));
    }

    public void decrementPositionTicks(int amount) {
        int newPosition = targetPositionTicks - amount * encoderDirection;
        moveToPositionTicks(Math.round(newPosition + adjustedZeroPosition));
    }

    public void moveToPositionInches(float positionInches) {
        positionInches = Math.max(0, Math.min(positionInches, LENGTH_IN));
        lastPositionInches = positionInches;
        int targetPosition = Math.round(positionInches * MOTOR_TICKS_PER_INCH * encoderDirection + adjustedZeroPosition);
        moveToPositionTicks(targetPosition);
    }

    public DcMotorEx getMotor() {
        return motor;
    }
}
