package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.ScorePixelsGroundCommand;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.TelemetryHandler;

@Autonomous(group = "test")
public class ScorePixelGroundTestOpMode extends LinearOpMode {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final TelemetryHandler telemetryHandler = new TelemetryHandler(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        ScoringMech scoringMech = new ScoringMech(hardwareMap, telemetryHandler);

        waitForStart();

        scheduler.schedule(scoringMech.scoreGround());

        while (!isStopRequested() && opModeIsActive()) {
            scheduler.run();
            telemetryHandler.update();
        }

        scheduler.reset();
    }
}
