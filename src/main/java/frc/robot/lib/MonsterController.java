// package com.vendor;
package frc.robot.lib;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class MonsterController implements MotorController {

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

    public void setPosition(int target) {
        _target = target;
        MonsterProtocol.instance.set_pos(_motorID, _target, 100);
    }

    public int getTarget() {
        return _target;
    }
    
}
