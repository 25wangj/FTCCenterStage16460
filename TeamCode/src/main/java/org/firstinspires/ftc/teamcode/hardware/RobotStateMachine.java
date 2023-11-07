package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.clawClosed;
import static org.firstinspires.ftc.teamcode.hardware.Lift.clawOpen;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.command.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
public class RobotStateMachine {
    public enum robotStates {
        INTAKE_CLOSED, INTAKE_OPEN, EJECT_CLOSED, EJECT_OPEN, DEPOSIT, RETRACT
    }
    public static StateMachine<robotStates> get(CommandOpMode opMode, Robot robot, robotStates state) {
        Subsystem[] subsystems = new Subsystem[] {robot.intake, robot.lift};
        StateMachineBuilder<robotStates> builder = new StateMachineBuilder<robotStates>(opMode)
                .addState(robotStates.values())
                .addTransition(robotStates.INTAKE_CLOSED, robotStates.EJECT_CLOSED, FnCommand.once(t -> {
                robot.intake.setRoller(rollerUp);
                robot.intake.setPower(intakeEject);}, subsystems))
                .addTransition(robotStates.EJECT_CLOSED, robotStates.INTAKE_CLOSED, FnCommand.once(t -> {
                robot.intake.setRoller(rollerDown);
                robot.intake.setPower(intakeClosed);}, subsystems))
                .addTransition(robotStates.INTAKE_OPEN, robotStates.EJECT_OPEN, FnCommand.once(t -> {
                robot.intake.setRoller(rollerUp);
                robot.intake.setPower(intakeEject);}, subsystems))
                .addTransition(robotStates.EJECT_OPEN, robotStates.INTAKE_OPEN, FnCommand.once(t -> {
                robot.intake.setRoller(rollerDown);
                robot.intake.setPower(intakeOpen);}, subsystems))
                .addTransition(robotStates.INTAKE_CLOSED, robotStates.INTAKE_OPEN, new SeqCommand(
                new WaitCommand(t -> {
                    robot.intake.setGate(gateOpen);
                    robot.intake.setPower(0.2);}, 0.5, subsystems),
                new WaitCommand(t -> robot.intake.setPower(0), 0.1, subsystems),
                FnCommand.once(t -> robot.intake.setPower(intakeOpen), subsystems)))
                .addTransition(robotStates.INTAKE_OPEN, robotStates.INTAKE_CLOSED, new WaitCommand(t -> {
                robot.intake.setGate(gateClosed);
                robot.intake.setPower(intakeClosed);}, 0.25, subsystems))
                .addTransition(robotStates.INTAKE_OPEN, robotStates.DEPOSIT, d -> new SeqCommand(
                new WaitCommand(t -> robot.intake.setGate(gatePush), 0.15, subsystems),
                new WaitCommand(t -> robot.intake.setGate(gateUp), 0.25, subsystems),
                FnCommand.once(t -> {
                    robot.lift.setClaw(clawClosed);
                    robot.intake.setPower(0);}, subsystems),
                robot.lift.goTo(d[0], d[1])))
                .addTransition(robotStates.DEPOSIT, robotStates.RETRACT, new SeqCommand(
                        new WaitCommand(t -> robot.lift.setClaw(clawOpen), 0.15, subsystems),
                        new WaitCommand(t -> robot.lift.setClaw(clawClosed), 0.1, subsystems),
                        new WaitCommand(t -> robot.lift.setClaw(clawOpen), 0.25, subsystems)))
                .addTransition(robotStates.RETRACT, robotStates.DEPOSIT, new WaitCommand(t ->
                robot.lift.setClaw(clawClosed), 0.25, subsystems))
                .addTransition(robotStates.RETRACT, robotStates.INTAKE_CLOSED, new SeqCommand(
                new ParCommand(robot.lift.goBack(), new WaitCommand(t -> {
                    robot.intake.setGate(gateOpen);
                    robot.intake.setRoller(rollerDown);}, 0.25, robot.intake)),
                new WaitCommand(t -> robot.intake.setGate(gateClosed), 0.25, subsystems),
                FnCommand.once(t -> robot.intake.setPower(intakeClosed), subsystems)))
                .addTransition(robotStates.RETRACT, robotStates.INTAKE_OPEN, new SeqCommand(
                new ParCommand(robot.lift.goBack(), new WaitCommand(t -> {
                    robot.intake.setGate(gateOpen);
                    robot.intake.setRoller(rollerDown);}, 0.25, robot.intake)),
                FnCommand.once(t -> robot.intake.setPower(intakeOpen), subsystems)));
        return builder.build(state);
    }
}