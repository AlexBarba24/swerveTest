// package com.vendor;
package frc.robot.lib;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class MonsterController implements MotorController {

    /** Board speed used by {@link #setPosition(int)} when no speed is supplied. */
    public static final int DEFAULT_POSITION_SPEED = 100;

    private int _motorID;
    private double _speed;
    private int _target;
    private int _inverted = 1;

    public MonsterController(int motorID) {
        if (MonsterProtocol.instance == null)
            MonsterProtocol.instance = new MonsterProtocol();
        _motorID = motorID;
        MonsterProtocol.instance.register_motor(_motorID);
    }

    @Override
    public void set(double speed) {
        _speed = speed;
        int setSpeed = (int) (speed*250.0);
        MonsterProtocol.instance.set_speed(_motorID, _inverted * setSpeed);
    }

    @Override
    public double get() {
        return _speed;
    }

    @Override
    public void setInverted(boolean isInverted) {
        _inverted = isInverted ? -1 : 1;
    }

    @Override
    public boolean getInverted() {
        return _inverted == -1;
    }

    @Override
    public void disable() {
        MonsterProtocol.instance.disable(_motorID);
    }

    @Override
    public void stopMotor() {
        set(0);
    }

    /**
     * Commands an absolute position move at {@link #DEFAULT_POSITION_SPEED}.
     *
     * @param target absolute encoder target in board counts
     */
    public void setPosition(int target) {
        setPosition(target, DEFAULT_POSITION_SPEED);
    }

    /**
     * Commands an absolute position move.
     *
     * @param target absolute encoder target in board counts
     * @param speed board speed to travel at, -250 to 250
     */
    public void setPosition(int target, int speed) {
        _target = target;
        MonsterProtocol.instance.set_pos(_motorID, _inverted * target, speed);
    }

    /**
     * Commands a position move relative to where the motor currently is, at
     * {@link #DEFAULT_POSITION_SPEED}.
     *
     * @param offset encoder counts to travel from the current position
     */
    public void setRelativePosition(int offset) {
        setRelativePosition(offset, DEFAULT_POSITION_SPEED);
    }

    /**
     * Commands a position move relative to where the motor currently is.
     *
     * @param offset encoder counts to travel from the current position
     * @param speed board speed to travel at, -250 to 250
     */
    public void setRelativePosition(int offset, int speed) {
        MonsterProtocol.instance.set_relative_pos(_motorID, _inverted * offset, speed);
    }

    /** Asks the board to report this motor's status on the next protocol flush. */
    public void requestStatus() {
        MonsterProtocol.instance.request_status(_motorID);
    }

    /**
     * Returns the last target given to {@link #setPosition(int, int)}, before inversion is
     * applied.
     *
     * @return the requested absolute target in board counts
     */
    public int getTarget() {
        return _target;
    }

    /**
     * Returns the CAN ID this controller drives, for log lines that need to identify a motor.
     *
     * @return the motor ID, 0 to 7
     */
    public int getMotorID() {
        return _motorID;
    }
}
