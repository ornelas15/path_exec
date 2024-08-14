
/** SampApp methods **/

#include "sampapp.h"

SampApp::SampApp(int &argc, char*argv[], char *robot_name) : QApplication(argc,argv)
{
    strncpy(rob_name, robot_name, 19);
    rob_name[19]='\0';

    beaconToFollow = 0; // start by finding target at beacon 0
}

/* Calculate the power of left and right motors */
void SampApp::DetermineAction(int beaconToFollow, double *lPow, double *rPow)
{
    static int counter=0;

    static double left, right, center;
    bool   beaconReady;
    static struct beaconMeasure beacon;
    static int    Ground;
    static bool   Collision;

    /*Access to values from Sensors - Only ReadSensors() gets new values */
    if(IsObstacleReady(LEFT))
        left      = GetObstacleSensor(LEFT);
    if(IsObstacleReady(RIGHT))
        right     = GetObstacleSensor(RIGHT);
    if(IsObstacleReady(CENTER))
        center    = GetObstacleSensor(CENTER);

    beaconReady = IsBeaconReady(beaconToFollow);
    if(beaconReady) {
        beacon    = GetBeaconSensor(beaconToFollow);
    }

    if(IsGroundReady())
        Ground    = GetGroundSensor();
    if(IsBumperReady())
        Collision = GetBumperSensor();

    if(center>4.5 || right>4.5 || left>4.5 || Collision) { /* Close Obstacle - Rotate */
        if(counter % 400 < 200) {
            *lPow=0.06;
            *rPow=-0.06; }
        else {
            *lPow=-0.06;
            *rPow=0.06; }
    }
    else if(right>1.5) { /* Obstacle Near - Avoid */
        *lPow=0.0;
        *rPow=0.05;
    }
    else if(left>1.5) {
        *lPow=0.05;
        *rPow=0.0;
    }
    else {
        if(beaconReady && beacon.beaconVisible) {
            if(beacon.beaconDir>20.0) { /* turn to Beacon */
                *lPow=0.0;
                *rPow=0.1;
            }
            else if(beacon.beaconDir<-20.0) {
                *lPow=0.1;
                *rPow=0.0;
            }
            else { /* Full Speed Ahead */
                *lPow=0.1;
                *rPow=0.1;
            }
        }
        else { /* Full Speed Ahead */
            *lPow=0.1;
            *rPow=0.1;
        }
    }

    counter++;
}

#define RUN        1
#define STOP       2
#define WAIT       3
#define RETURN     4
#define FINISHED   5

void SampApp::act(void)
{
    static int state=STOP,stoppedState=RUN;
    double lPow, rPow;

    ReadSensors();

    if(GetFinished()) /* Simulator has received Finish() or Robot Removed */
    {
        printf("%s Exiting %d\n",rob_name, GetTime());
        exit(0);
        return;
    }
    if(state==STOP && GetStartButton()) state=stoppedState;  /* Start     */
    if(state!=STOP && GetStopButton())  {
        stoppedState=state;
        state=STOP; /* Interrupt */
    }

    switch (state) {
        case RUN:    /* Go */
            if( GetVisitingLed() ) state = WAIT;
            if(GetGroundSensor()==0) {         /* Visit Target */
                SetVisitingLed(true);
                printf("%s visited target at %d\n", rob_name, GetTime());
            }

            else {
                DetermineAction(0,&lPow,&rPow);
                DriveMotors(lPow,rPow);
            }
            break;
        case WAIT: /* Wait for others to visit target */
            SetReturningLed(true);
            if(GetVisitingLed()) SetVisitingLed(false);
            if(GetReturningLed()) state = RETURN;
            DriveMotors(0.0,0.0);
            break;
        case RETURN: /* Return to home area */
            if(GetVisitingLed()) SetVisitingLed(false);
            SetReturningLed(false);

            // Wander
            DetermineAction(1,&lPow,&rPow);
            DriveMotors(lPow,rPow);

            break;

    }
}

void SampApp::setMap(char lmap[][CELLCOLS*2-1])
{
    memcpy(this->lmap,lmap,(CELLROWS*2-1)*(CELLCOLS*2-1));
}

void SampApp::printMap()
{
    for(int r=CELLROWS * 2 - 2; r >=0 ; r--) {
        for(int c=0; c < CELLCOLS * 2 - 1; c++) {
            printf("%c",lmap[r][c]);
        }
        printf("\n");
    }
}
