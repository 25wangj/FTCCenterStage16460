package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.movement.Vec;
import org.firstinspires.ftc.teamcode.sensors.RisingEdgeDetector;
@TeleOp(name = "OneDriver")
public class TeleopOneDriver extends CommandOpMode {
    private Robot robot;
    private double lastLiftPos = liftLow;
    private double lastArmPos = 0;
    @Override
    public void initOpMode() {
        robot = new Robot(this, false);
        robot.drive.setHeading(lastPose.h + lastSide * PI / 2);
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.ps, FnCommand.once(t -> robot.drive.setHeading(0))));
        scheduler.schedule(FnCommand.repeat(t -> {
            if (gamepad1.right_trigger > 0.9) {
                if (robot.stateMachine.state() == INTAKE_OPEN) {
                    robot.stateMachine.transition(EJECT_OPEN);
                } else if (robot.stateMachine.state() == INTAKE_CLOSED) {
                    robot.stateMachine.transition(EJECT_CLOSED);
                }
            } else if (robot.stateMachine.state() == EJECT_OPEN) {
                robot.stateMachine.transition(INTAKE_OPEN);
            } else if (robot.stateMachine.state() == EJECT_CLOSED) {
                robot.stateMachine.transition(INTAKE_CLOSED);
            }
            if (gamepad1.left_trigger > 0.1) {
                if (robot.stateMachine.state() == INTAKE_CLOSED || robot.stateMachine.state() == INTAKE_OPEN) {
                    robot.intake.setRoller(scale(gamepad1.left_trigger, 0.1, 1, rollerDown, rollerUp));
                }
            } else if (robot.stateMachine.state() == INTAKE_CLOSED || robot.stateMachine.state() == INTAKE_OPEN) {
                robot.intake.setRoller(rollerDown);
            }
            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                if (robot.stateMachine.state() == INTAKE_OPEN) {
                    scheduler.schedule(robot.intake.twiddle());
                } else if (robot.stateMachine.state() == DEPOSIT || robot.stateMachine.state() == RETRACT) {
                    //scheduler.schedule(robot.lift.adjust(liftAdjust(gamepad1.dpad_up, gamepad1.dpad_down),
                    //        armAdjust(gamepad1.dpad_right, gamepad1.dpad_left)));
                }
            }
            Vec p = new Vec(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotate(-robot.drive.getHeading());
            double turn = -gamepad1.right_stick_x;
            if (p.norm() + abs(turn) < 0.05) {
                robot.drive.setPowers(0, 0, 0);
            } else {
                robot.drive.setPowers(p.x, p.y, turn);
            }
        }, robot.drive));
        scheduler.addListener(
                RisingEdgeDetector.listen(() -> gamepad1.right_bumper, FnCommand.once(t -> {
                    if (robot.stateMachine.state() == INTAKE_CLOSED) {
                        robot.stateMachine.transition(INTAKE_OPEN);
                    } else if (robot.stateMachine.state() == INTAKE_OPEN) {
                        robot.stateMachine.transition(DEPOSIT, lastLiftPos, lastArmPos);
                    } else if (robot.stateMachine.state() == DEPOSIT) {
                        robot.stateMachine.transition(RETRACT);
                    } else if (robot.stateMachine.state() == RETRACT) {
                        robot.stateMachine.transition(INTAKE_OPEN);
                    }})),
                RisingEdgeDetector.listen(() -> gamepad1.left_bumper, FnCommand.once(t -> {
                    if (robot.stateMachine.state() == INTAKE_OPEN) {
                        robot.stateMachine.transition(INTAKE_CLOSED);
                    } else if (robot.stateMachine.state() == RETRACT) {
                        robot.stateMachine.transition(DEPOSIT);
                    }})),
                RisingEdgeDetector.listen(() -> (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y), FnCommand.once(t -> {
                    double liftPos = liftPos(gamepad1.b, gamepad1.x, gamepad1.y);
                    double armPos = armPos(gamepad1.left_trigger > 0.1, gamepad1.right_trigger > 0.1);
                    if (robot.stateMachine.state() == INTAKE_OPEN) {
                        if (robot.stateMachine.transition(DEPOSIT, liftPos, armPos)) {
                            lastLiftPos = liftPos;
                            lastArmPos = armPos;
                        }
                    } else if (robot.stateMachine.state() == DEPOSIT || robot.stateMachine.state() == RETRACT) {
                        if (scheduler.schedule(robot.lift.goTo(liftPos, armPos))) {
                            lastLiftPos = liftPos;
                            lastArmPos = armPos;}}})));
    }
    private static double liftPos(boolean b, boolean x, boolean y) {
        if (b) {
            return 720;
        } else if (y) {
            return 1220;
        } else if (x) {
            return liftHigh;
        }
        return 220;
    }
    private static double armPos(boolean left, boolean right) {
        if (left) {
            return armLeft;
        } else if (right) {
            return armRight;
        }
        return 0;
    }
    private static double liftAdjust(boolean up, boolean down) {
        if (up) {
            return 10;
        } else if (down) {
            return -10;
        }
        return 0;
    }
    private static double armAdjust(boolean right, boolean left) {
        if (right) {
            return 5;
        } else if (left) {
            return -5;
        }
        return 0;
    }
}
