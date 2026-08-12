package frc.robot.lib;

import edu.wpi.first.hal.CANAPITypes;
import edu.wpi.first.wpilibj.CAN;

/** FRC CAN interface for one eight-motor Monster controller board. */
public final class MonsterProtocol {
    private static final int MOTOR_COUNT = 8;
    private static final int MOTOR_ID_BASE = 0;
    private static final int MAX_SPEED = 250;

    // FRC API IDs are 10 bits: (API class << 4) | API index.
    private static final int API_STOP = 0x000;
    private static final int API_DISABLE = 0x001;
    private static final int API_MOVE_SPEED = 0x010;
    private static final int API_MOVE_ABSOLUTE = 0x030;
    private static final int API_MOVE_RELATIVE = 0x031;
    private static final int API_STATUS = 0x050;
    private static final int API_ENCODER_REQUEST = 0x051;

    private enum CommandType {
        STOP(API_STOP, false),
        DISABLE(API_DISABLE, false),
        MOVE_SPEED(API_MOVE_SPEED, true),
        MOVE_ABSOLUTE(API_MOVE_ABSOLUTE, true),
        MOVE_RELATIVE(API_MOVE_RELATIVE, true),
        STATUS(API_STATUS, false),
        ENCODER_REQUEST(API_ENCODER_REQUEST, false);

        final int apiId;
        final boolean hasPayload;

        CommandType(int apiId, boolean hasPayload) {
            this.apiId = apiId;
            this.hasPayload = hasPayload;
        }
    }

    private static final class Command {
        final CommandType type;
        final int position;
        final int speed;

        Command(CommandType type, int position, int speed) {
            this.type = type;
            this.position = position;
            this.speed = speed;
        }
    }

    private final Command[] commandBuffer = new Command[MOTOR_COUNT];
    private final CAN[] canHandles = new CAN[MOTOR_COUNT];

    public static MonsterProtocol instance;

    public void register_motor(int motorID) {
        validateMotorID(motorID);
        if (canHandles[motorID] != null) {
            return;
        }

        canHandles[motorID] = new CAN(
            MOTOR_ID_BASE + motorID,
            CANAPITypes.CANManufacturer.kTeamUse.id,
            CANAPITypes.CANDeviceType.kMotorController.id
        );
    }

    public void set_speed(int motorID, int speed) {
        validateMotorID(motorID);
        validateSpeed(speed);

        // The board defines speed zero as equivalent to STOP. Use the dedicated
        // class-0 command so it wins arbitration over queued motion traffic.
        commandBuffer[motorID] = speed == 0
            ? new Command(CommandType.STOP, 0, 0)
            : new Command(CommandType.MOVE_SPEED, 0, speed);
    }

    /** Queue an absolute-position move. */
    public void set_pos(int motorID, int position, int speed) {
        queueMotion(motorID, CommandType.MOVE_ABSOLUTE, position, speed);
    }

    /** Queue a relative-position move. */
    public void set_relative_pos(int motorID, int offset, int speed) {
        queueMotion(motorID, CommandType.MOVE_RELATIVE, offset, speed);
    }

    /** Ramp to a stop while leaving the motor driver energized. */
    public void stop(int motorID) {
        validateMotorID(motorID);
        commandBuffer[motorID] = new Command(CommandType.STOP, 0, 0);
    }

    /** Stop and de-energize the motor driver. */
    public void disable(int motorID) {
        validateMotorID(motorID);
        commandBuffer[motorID] = new Command(CommandType.DISABLE, 0, 0);
    }

    public void request_status(int motorID) {
        validateMotorID(motorID);
        commandBuffer[motorID] = new Command(CommandType.STATUS, 0, 0);
    }

    /** Ask the board to publish its four encoder values immediately. */
    public void request_encoders() {
        if (canHandles[0] == null) {
            throw new IllegalStateException("Motor 0 must be registered before requesting encoders");
        }
        commandBuffer[0] = new Command(CommandType.ENCODER_REQUEST, 0, 0);
    }

    public void periodic() {
        for (int motorID = 0; motorID < MOTOR_COUNT; motorID++) {
            Command command = commandBuffer[motorID];
            CAN can = canHandles[motorID];
            if (command == null || can == null) {
                continue;
            }

            byte[] payload = command.type.hasPayload
                ? encodePayload(command.position, command.speed)
                : new byte[0];
            can.writePacket(payload, command.type.apiId);
            commandBuffer[motorID] = null;
        }
    }

    private void queueMotion(int motorID, CommandType type, int position, int speed) {
        validateMotorID(motorID);
        validateSpeed(speed);
        commandBuffer[motorID] = new Command(type, position, speed);
    }

    private static byte[] encodePayload(int position, int speed) {
        return new byte[] {
            (byte) position,
            (byte) (position >>> 8),
            (byte) (position >>> 16),
            (byte) (position >>> 24),
            (byte) speed,
            (byte) (speed >>> 8),
            (byte) (speed >>> 16),
            (byte) (speed >>> 24)
        };
    }

    private static void validateMotorID(int motorID) {
        if (motorID < 0 || motorID >= MOTOR_COUNT) {
            throw new IllegalArgumentException("Motor ID must be between 0 and 7: " + motorID);
        }
    }

    private static void validateSpeed(int speed) {
        if (speed < -MAX_SPEED || speed > MAX_SPEED) {
            throw new IllegalArgumentException("Speed must be between -250 and 250: " + speed);
        }
    }
}
