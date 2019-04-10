package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmGoCargo;
import frc.robot.commands.ArmGoLow;
import frc.robot.commands.ArmGoRocket;
import frc.robot.commands.ArmManualDown;
import frc.robot.commands.ArmManualUp;
import frc.robot.commands.ArmSetAngle;
import frc.robot.commands.ArmZeroReset;
import frc.robot.commands.ClimberSet;
import frc.robot.commands.ClimberSetOverride;
import frc.robot.commands.DriverLowerGroup;
import frc.robot.commands.DriverRaiseGroup;
import frc.robot.commands.IntakeBoth;
import frc.robot.commands.IntakeLower;
import frc.robot.commands.IntakeRaise;
import frc.robot.commands.IntakeToggle;
import frc.robot.commands.MothClose;
import frc.robot.commands.MothOpen;
import frc.robot.commands.MothToggle;
import frc.robot.commands.OuttakeBoth;
import frc.robot.commands.VisionHatchPlacer;
import frc.robot.commands.VisionHoldOnTarget;

public class LaunchpadManager {

    double watchdogTimerSeconds = 0.1;
    final String tableName = "Launchpad";
    boolean btns[][] = new boolean[9][9];

    boolean isConnected = false;
    double lastReceivedPing = 0;
    boolean lastPingValue = false;

    NetworkTableInstance nt;
    NetworkTable table;
    NetworkTableEntry pingEntryRio;
    NetworkTableEntry pingEntryLaunchpad;

    public LaunchpadManager() {
        nt = NetworkTableInstance.getDefault();
        table = nt.getTable(tableName);
        pingEntryRio = table.getEntry("pingValueRio");
        pingEntryLaunchpad = table.getEntry("pingValueLaunchpad");
    }

    public void periodicLoop() {
        // get ping value posted by launchpad
        boolean launchpadPingValue = pingEntryLaunchpad.getBoolean(false);

        // check for ping
        if(lastPingValue != launchpadPingValue) {
            lastPingValue = launchpadPingValue;
            lastReceivedPing = Timer.getFPGATimestamp();
        }
        
        // decide if connected or not
        if(Timer.getFPGATimestamp() - lastReceivedPing > watchdogTimerSeconds) {
            isConnected = false;
        }
        SmartDashboard.putBoolean("Launchpad Connection", isConnected);

        // send ping
        pingEntryRio.setBoolean(!pingEntryRio.getBoolean(false));

        // get buttons
        for(int r = 0; r < 9; r++) {
            for(int c = 0; c < 9; c++) {
                String key = r + ":" + c;
                btns[r][c] = isConnected ? table.getEntry(key).getBoolean(false) : false;
            }
        }

        // connection button test
        if(btns[8][8]) SmartDashboard.putBoolean("Launchpad Button Test", true);
        else SmartDashboard.putBoolean("Launchpad Button Test", false);
    }

    public void disabledLoop() {
        
    }

    public void teleopLoop() {
        if(btns[0][0]) climberOpenOverride.start();
        else climberOpenOverride.cancel();

        if(btns[0][1]) climberCloseOverride.start();
        else climberCloseOverride.cancel();

        if(btns[0][2]) armResetZero.start();
        else armResetZero.cancel();

        if(btns[0][6]) armDownGroup.start();

        if(btns[0][7]) armUpGroup.start();

        if(btns[1][0]) climberOpen.start();
        else climberOpen.cancel();

        if(btns[1][1]) climberClose.start();
        else climberClose.cancel();

        if(btns[1][4]) limelightAuto.start();
        else limelightAuto.cancel();

        if(btns[1][5]) limelightHold.start();
        else limelightHold.cancel();

        if(btns[1][6]) mothClose.start();

        if(btns[1][7]) mothOpen.start();

        if(btns[1][8]) mothToggle.start();

        if(btns[2][4]) outtakeBoth.start();
        else outtakeBoth.cancel();

        if(btns[2][5]) intakeBoth.start();
        else intakeBoth.cancel();

        if(btns[2][6]) intakeDown.start();

        if(btns[2][7]) intakeUp.start();

        if(btns[2][8]) intakeToggle.start();

        if(btns[3][6]) armManualDown.start();
        else armManualDown.cancel();

        if(btns[3][7]) armManualUp.start();
        else armManualUp.cancel();

        if(btns[3][8]) armSetCargo.start();
        else armSetCargo.cancel();

        if(btns[4][8]) armSetRocket.start();
        else armSetRocket.cancel();

        if(btns[5][8]) armSetLow.start();
        else armSetLow.cancel();
    }

    /*

    Mappings:
    0:0 climber open override  --  while
    0:1 climber close override  --  while
    0:2 arm reset zero  --  while
    0:4 cancel auto  --  function
    0:6 arm down group  --  when
    0:7 arm up group  --  when
    1:0 climber open  --  while
    1:1 climber close  --  while
    1:4 limelight auto  --  while
    1:5 limelight hold  --  while
    1:6 moth close  --  when
    1:7 moth open  --  when
    1:8 moth toggle  --  when
    2:4 outtake both  --  while
    2:5 intake both  --  while
    2:6 intake down  --  when
    2:7 intake up  --  when
    2:8 intake toggle  --  when
    3:6 arm manual down  --  while
    3:7 arm manual up  --  while
    3:8 arm set cargo  --  when
    4:8 arm set rocket  --  when
    5:8 arm set low  --  when

    */

    // commands
    Command climberOpenOverride = new ClimberSetOverride(0.3);
    Command climberCloseOverride = new ClimberSetOverride(-0.3);
    Command armResetZero = new ArmZeroReset();
    Command armDownGroup = new DriverLowerGroup(false);
    Command armUpGroup = new DriverRaiseGroup();
    Command climberOpen = new ClimberSet(0.5,.3);
    Command climberClose = new ClimberSet(-0.5,0);
    Command limelightAuto = new VisionHatchPlacer();
    Command limelightHold = new VisionHoldOnTarget();
    Command mothClose = new MothClose();
    Command mothOpen = new MothOpen();
    Command mothToggle = new MothToggle();
    Command outtakeBoth = new OuttakeBoth();
    Command intakeBoth = new IntakeBoth(false);
    Command intakeDown = new IntakeLower();
    Command intakeUp = new IntakeRaise();
    Command intakeToggle = new IntakeToggle();
    Command armManualDown = new ArmManualDown();
    Command armManualUp = new ArmManualUp();
    Command armSetCargo = new ArmGoCargo();
    Command armSetRocket = new ArmGoRocket();
    Command armSetLow = new ArmGoLow();

}