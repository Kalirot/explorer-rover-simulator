/* Coordinate-Based Intelligent Line Following Robot Simulator
 * ANSI C / Turbo C 2.0 / Turbo C++ 3.0, VGA 640x480, small model.
 * Set BGI directory below, or pass it as the first program argument.
 * HEADLESS builds exercise the SAME controller without graphics.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#ifndef HEADLESS
#include <graphics.h>
#include <conio.h>
#include <dos.h>
#endif

#define MAX_POINTS 30
#define XMIN 40
#define XMAX 430
#define YMIN 110
#define YMAX 395
#define PI 3.14159265358979323846
#define LINE_RADIUS 3.0
#define SENSOR_RADIUS 1.5
#define BGI_DIR "C:\\TURBOC3\\BGI"

double xPoints[MAX_POINTS], yPoints[MAX_POINTS];
long worldX[MAX_POINTS], worldY[MAX_POINTS];
int pointCount;
double viewScale=1.0, viewLeft=0.0, viewTop=0.0;
int segment, sensorOn[3], finished, turns, sharpTurns;
double robotX, robotY, heading, sensorX[3], sensorY[3];
double pathLength, travelled, completedLength, speed = 2.0;
char motion[24];

/* Uniform world-to-screen mapping; user coordinates are never clipped. */
void fitPath(void)
{
    int i;
    double minX=0,maxX=0,minY=0,maxY=0,spanX,spanY,sx,sy;
    for(i=0;i<pointCount;++i) {
        if(worldX[i]<minX)minX=worldX[i];
        if(worldX[i]>maxX)maxX=worldX[i];
        if(worldY[i]<minY)minY=worldY[i];
        if(worldY[i]>maxY)maxY=worldY[i];
    }
    spanX=maxX-minX;spanY=maxY-minY;
    sx=spanX>0 ? (XMAX-XMIN)/spanX : 1.0e30;
    sy=spanY>0 ? (YMAX-YMIN)/spanY : 1.0e30;
    viewScale=sx<sy?sx:sy;
    if(spanX==0 && spanY==0)viewScale=1;
    viewLeft=XMIN+((XMAX-XMIN)-spanX*viewScale)/2-minX*viewScale;
    viewTop=YMIN+((YMAX-YMIN)-spanY*viewScale)/2+maxY*viewScale;
    for(i=0;i<pointCount;++i) {
        xPoints[i]=viewLeft+worldX[i]*viewScale;
        yPoints[i]=viewTop-worldY[i]*viewScale;
    }
}

double userX(void) {return (robotX-viewLeft)/viewScale;}
double userY(void) {return (viewTop-robotY)/viewScale;}

double clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double calculateDistance(double x1, double y1, double x2, double y2)
{
    double dx = x2-x1, dy = y2-y1;
    return sqrt(dx*dx+dy*dy);
}

double wrapAngle(double a)
{
    while (a > PI) a -= 2.0*PI;
    while (a < -PI) a += 2.0*PI;
    return a;
}

double calculateAngle(int s)
{
    return atan2((double)yPoints[s+1]-yPoints[s],
                 (double)xPoints[s+1]-xPoints[s]);
}

/* Distance to a FINITE segment, including its two end caps. */
double distanceToSegment(double x, double y, int s)
{
    double dx = xPoints[s+1]-xPoints[s];
    double dy = yPoints[s+1]-yPoints[s];
    double len2 = dx*dx+dy*dy;
    double t;
    if (len2 == 0.0)
        return calculateDistance(x,y,xPoints[s],yPoints[s]);
    t = ((x-xPoints[s])*dx+(y-yPoints[s])*dy)/len2;
    t = clamp(t,0.0,1.0);
    return calculateDistance(x,y,xPoints[s]+t*dx,yPoints[s]+t*dy);
}

void detectSensors(void)
{
    int i, s;
    double lateral;
    for (i=0; i<3; ++i) {
        /* Screen Y grows downward: left normal is (sin h,-cos h). */
        lateral = (1-i)*5.0;
        sensorX[i] = robotX+6.0*cos(heading)+lateral*sin(heading);
        sensorY[i] = robotY+6.0*sin(heading)-lateral*cos(heading);
        sensorOn[i] = 0;
        for (s=0; s<pointCount-1; ++s)
            if (distanceToSegment(sensorX[i],sensorY[i],s)
                <= LINE_RADIUS+SENSOR_RADIUS) sensorOn[i] = 1;
    }
}

void analyzePath(void)
{
    int s;
    double a;
    pathLength = 0.0;
    turns = sharpTurns = 0;
    for (s=0; s<pointCount-1; ++s) {
        pathLength += calculateDistance(xPoints[s],yPoints[s],
                                        xPoints[s+1],yPoints[s+1]);
        if (s > 0) {
            a = fabs(wrapAngle(calculateAngle(s)-calculateAngle(s-1)));
            if (a > 10.0*PI/180.0) ++turns;
            if (a >= 70.0*PI/180.0) ++sharpTurns;
        }
    }
}

void resetRobot(void)
{
    segment = finished = 0;
    robotX = xPoints[0]; robotY = yPoints[0];
    heading = calculateAngle(0);
    travelled = completedLength = 0.0;
    strcpy(motion,"FOLLOWING");
    detectSensors();
}

/* Hybrid controller: coordinates select the route; sensors trim steering.
 * A turn too large to drive safely is performed in place. No waypoint snap.
 * Arrival uses a final distance-limited step, never a position assignment.
 */
void updateRobot(void)
{
    double dx, dy, distance, desired, error, correction, step;
    int nearCorner;
    if (finished) return;
    dx = xPoints[segment+1]-robotX;
    dy = yPoints[segment+1]-robotY;
    distance = sqrt(dx*dx+dy*dy);
    if (distance < 0.00000000001) {
        completedLength += calculateDistance(xPoints[segment],yPoints[segment],
                                    xPoints[segment+1],yPoints[segment+1]);
        if (segment == pointCount-2) {
            finished = 1;
            strcpy(motion,"COMPLETED");
        } else {
            ++segment;
            strcpy(motion,"TURNING");
        }
        detectSensors();
        return;
    }
    desired = atan2(dy,dx);
    error = wrapAngle(desired-heading);
    detectSensors();
    correction = 0.0;
    nearCorner = distance < 14.0;
    if (!nearCorner && fabs(error) < 0.35) {
        if (!sensorOn[1] && sensorOn[0] && !sensorOn[2]) correction = -0.055;
        if (!sensorOn[1] && sensorOn[2] && !sensorOn[0]) correction = 0.055;
    }
    if (!sensorOn[0] && !sensorOn[1] && !sensorOn[2]) {
        /* Guided recovery: rotate toward the active target, slowly advance. */
        strcpy(motion,"RECOVERING");
    } else strcpy(motion,"FOLLOWING");
    if (fabs(error) > 0.30) {
        heading = wrapAngle(heading+clamp(error,-0.12,0.12));
        strcpy(motion,"TURNING");
        detectSensors();
        return;
    }
    heading = wrapAngle(heading+clamp(error,-0.09,0.09)+correction);
    /* Near a point, remove sensor bias and align before the last step. */
    if (nearCorner && fabs(wrapAngle(desired-heading)) <= 0.09)
        heading = desired;
    step = speed;
    if (nearCorner) step *= 0.55;
    if (!sensorOn[0] && !sensorOn[1] && !sensorOn[2]) step *= 0.45;
    if (step > distance) step = distance;
    robotX += step*cos(heading);
    robotY += step*sin(heading);
    travelled += step;
    detectSensors();
}

double completion(void)
{
    double len, dx, dy, t;
    if (finished) return 100.0;
    dx = xPoints[segment+1]-xPoints[segment];
    dy = yPoints[segment+1]-yPoints[segment];
    len = sqrt(dx*dx+dy*dy);
    t = ((robotX-xPoints[segment])*dx+(robotY-yPoints[segment])*dy)/(len*len);
    return 100.0*(completedLength+clamp(t,0.0,1.0)*len)/pathLength;
}

int validateCoordinates(int x, int y, int used)
{
    int i;
    if (x<XMIN || x>XMAX || y<YMIN || y>YMAX) return 0;
    for (i=0; i<used; ++i)
        if (xPoints[i]==x && yPoints[i]==y) return -1;
    return 1;
}

void samplePath(void)
{
    int i;
    int xs[8] = {65,165,245,325,375,275,350,410};
    int ys[8] = {355,355,285,285,185,230,335,155};
    pointCount = 8;
    for (i=0;i<pointCount;++i) {
        worldX[i]=xs[i]-xs[0]; worldY[i]=ys[0]-ys[i];
    }
    fitPath();
}

#ifndef HEADLESS
int frameCount;
double trailX[800], trailY[800];
int trailCount;
/* A 33x33 BGI patch is small enough for a 16-bit data segment.
 * Save just the scenery beneath the rover, never a full VGA frame.
 */
unsigned char roverBackground[2048];
int backgroundX,backgroundY,backgroundValid;

void textAt(int x, int y, int color, char *s)
{
    setcolor(color); outtextxy(x,y,s);
}

void screenTitle(char *subtitle)
{
    setbkcolor(BLACK); cleardevice();
    setfillstyle(SOLID_FILL,BLUE); bar(0,0,639,48);
    textAt(16,10,WHITE,"COORDINATE-BASED INTELLIGENT LINE FOLLOWING ROBOT");
    textAt(16,29,LIGHTCYAN,"ROBOT SIMULATOR  /  USER-DEFINED PATH + VIRTUAL SENSORS");
    textAt(16,61,YELLOW,subtitle);
}

/* Bounded graphical numeric editor: no scanf overflow or leftover newline. */
void inputMessage(char *message)
{
    setfillstyle(SOLID_FILL,BLACK);
    bar(30,320,620,350);
    textAt(35,328,LIGHTRED,message);
}

int readNumber(char *label, long lo, long hi, long *value)
{
    char buf[16], range[80], display[20], error[80];
    int n=0, c, maxDigits;
    long v;
    buf[0]='\0';
    setfillstyle(SOLID_FILL,BLACK); bar(30,210,620,350);
    if(lo<0)sprintf(range,"Enter %s (positive, negative or zero):",label);
    else sprintf(range,"Enter %s (%ld to %ld):",label,lo,hi);
    textAt(35,220,WHITE,range);
    sprintf(error,"%ld",hi); maxDigits=(int)strlen(error);
    setcolor(CYAN); rectangle(35,244,220,278);
    textAt(35,290,LIGHTGRAY,"Type ONE number, then press ENTER to continue.");
    textAt(35,305,LIGHTGRAY,"BACKSPACE = edit     ESC = cancel");
    for (;;) {
        setfillstyle(SOLID_FILL,BLACK); bar(37,246,218,276);
        sprintf(display,"%s_",buf);
        textAt(45,255,LIGHTCYAN,display);
        c=getch();
        if (c==0 || c==224) { getch(); continue; }
        if (c==27) return 0;
        if (c==8) {
            if(n>0)buf[--n]='\0';
            inputMessage("");
        } else if(c=='-' && n==0 && lo<0) {
            buf[n++]='-';buf[n]='\0';inputMessage("");
        } else if (c>='0' && c<='9') {
            if(n<maxDigits+(buf[0]=='-'?1:0)) {
                buf[n++]=(char)c; buf[n]='\0'; inputMessage("");
            } else inputMessage("Too many digits. Use BACKSPACE to edit your number.");
        } else if (c==13) {
            if(n==0 || (n==1 && buf[0]=='-'))inputMessage("Please type a number first, then press ENTER.");
            else {
                v=strtol(buf,NULL,10);
                if (v>=lo && v<=hi) { *value=v; return 1; }
                sprintf(error,"Please enter a number from %ld to %ld.",lo,hi);
                inputMessage(error);
                n=0; buf[0]='\0';
            }
        } else {
            inputMessage("Type a whole number, then press ENTER.");
        }
    }
}

int choosePointCount(void)
{
    int key;
    long count;
    screenTitle("STEP 1 / SELECT HOW MANY POINTS YOU WANT");
    textAt(80,120,YELLOW,"Choose your path size:");
    textAt(100,165,WHITE,"1. 3 points");
    textAt(100,200,WHITE,"2. 5 points");
    textAt(100,235,WHITE,"3. 10 points");
    textAt(100,270,WHITE,"4. Choose another number");
    textAt(80,325,LIGHTGREEN,"Start (0,0) is included in your selected total.");
    textAt(80,355,LIGHTGRAY,"Press 1, 2, 3 or 4. ESC returns to the menu.");
    for(;;) {
        key=getch();
        if(key==0 || key==224) {getch();continue;}
        if(key==27)return 0;
        if(key=='1') {pointCount=3;return 1;}
        if(key=='2') {pointCount=5;return 1;}
        if(key=='3') {pointCount=10;return 1;}
        if(key=='4') {
            screenTitle("CHOOSE YOUR OWN POINT COUNT");
            textAt(35,125,WHITE,"Type how many total points you want, then press ENTER.");
            textAt(35,155,LIGHTGREEN,"The starting point (0,0) is included.");
            if(!readNumber("total points",2,MAX_POINTS,&count))return 0;
            pointCount=(int)count;return 1;
        }
    }
}

int getCoordinates(void)
{
    int i, j, valid;
    long x, y;
    char label[80];
    if(!choosePointCount())return 0;
    worldX[0]=worldY[0]=0;
    for (i=1;i<pointCount;++i) {
        do {
            screenTitle("USER INPUT -> CUSTOM COORDINATE PATH");
            sprintf(label,"YOUR %d-POINT PATH / ENTER POINT %d OF %d",pointCount,i+1,pointCount);
            textAt(35,125,YELLOW,label);
            textAt(35,100,LIGHTGREEN,"POINT 1 SAVED AUTOMATICALLY: (0,0)");
            textAt(35,150,LIGHTGRAY,"From (0,0): +X right, +Y up. Track fits automatically.");
            textAt(35,175,WHITE,"Enter X first (example: 100), then press ENTER.");
            if (!readNumber("X",-999999999L,999999999L,&x)) return 0;
            setfillstyle(SOLID_FILL,BLACK); bar(30,170,620,205);
            sprintf(label,"X saved: %ld. Now enter Y (example: 300).",x);
            textAt(35,175,LIGHTGREEN,label);
            if (!readNumber("Y",-999999999L,999999999L,&y)) return 0;
            valid=1;
            for(j=0;j<i;++j)
                if(worldX[j]==x && worldY[j]==y)valid=-1;
            if (valid!=1) {
                textAt(35,355,LIGHTRED,"ERROR: Duplicate coordinate. Press a key to re-enter.");
                getch();
            }
        } while (valid!=1);
        worldX[i]=x; worldY[i]=y;
    }
    fitPath();
    return 1;
}

void drawGrid(void)
{
    int x,y;
    char b[32];
    setcolor(DARKGRAY);
    for (x=50;x<=450;x+=50) line(x,90,x,415);
    for (y=100;y<=400;y+=50) line(20,y,450,y);
    rectangle(20,90,450,415);
    for (x=50;x<=400;x+=50) {
        sprintf(b,"%.0f",(x-viewLeft)/viewScale); textAt(x-10,420,LIGHTGRAY,b);
    }
    for (y=100;y<=400;y+=50) {
        sprintf(b,"%.0f",(viewTop-y)/viewScale); textAt(21,y+3,LIGHTGRAY,b);
    }
}

void drawPath(void)
{
    int i, j, mx, my;
    double a;
    setcolor(LIGHTGRAY);
    setlinestyle(SOLID_LINE,0,THICK_WIDTH);
    for (i=0;i<pointCount-1;++i) {
        line(xPoints[i],yPoints[i],xPoints[i+1],yPoints[i+1]);
        /* Parallel strokes produce a roughly 6px geometric track. */
        a=calculateAngle(i);
        for (j=-2;j<=2;++j)
            line(xPoints[i]+(int)(j*sin(a)),yPoints[i]-(int)(j*cos(a)),
                 xPoints[i+1]+(int)(j*sin(a)),yPoints[i+1]-(int)(j*cos(a)));
    }
    setlinestyle(SOLID_LINE,0,NORM_WIDTH);
    for (i=0;i<pointCount-1;++i) {
        a=calculateAngle(i);
        mx=(xPoints[i]+xPoints[i+1])/2; my=(yPoints[i]+yPoints[i+1])/2;
        setcolor(YELLOW);
        line(mx,my,mx-(int)(8*cos(a-0.5)),my-(int)(8*sin(a-0.5)));
        line(mx,my,mx-(int)(8*cos(a+0.5)),my-(int)(8*sin(a+0.5)));
    }
}

void drawPoints(void)
{
    int i, c, lx, ly;
    char b[40];
    for (i=0;i<pointCount;++i) {
        c=i==0 ? LIGHTGREEN : (i==pointCount-1 ? LIGHTRED : LIGHTCYAN);
        setcolor(c); circle(xPoints[i],yPoints[i],5);
        sprintf(b,"P%d (%ld,%ld)",i+1,worldX[i],worldY[i]);
        lx=xPoints[i]-35; ly=yPoints[i]+10;
        if (lx<22) lx=22;
        if (lx+(int)strlen(b)*8>448) lx=448-(int)strlen(b)*8;
        if (ly>402) ly=yPoints[i]-16;
        textAt(lx,ly,c,b);
    }
    textAt(xPoints[0]-18,yPoints[0]-20,LIGHTGREEN,"START");
    textAt(xPoints[pointCount-1]-23,yPoints[pointCount-1]-20,LIGHTRED,"FINISH");
}

/* Draw a rectangle in robot-local forward/left coordinates. */
void robotBox(double f1,double l1,double f2,double l2,int color)
{
    int p[10],i;
    double f[4],l[4];
    f[0]=f1;l[0]=l1; f[1]=f2;l[1]=l1;
    f[2]=f2;l[2]=l2; f[3]=f1;l[3]=l2;
    for(i=0;i<4;++i) {
        p[2*i]=(int)(robotX+f[i]*cos(heading)+l[i]*sin(heading));
        p[2*i+1]=(int)(robotY+f[i]*sin(heading)-l[i]*cos(heading));
    }
    p[8]=p[0];p[9]=p[1];
    setcolor(color);setfillstyle(SOLID_FILL,color);fillpoly(4,p);
}

void drawSensors(void)
{
    int i,c;
    for(i=0;i<3;++i) {
        c=sensorOn[i] ? LIGHTGREEN : LIGHTRED;
        setcolor(c);setfillstyle(SOLID_FILL,c);
        fillellipse((int)sensorX[i],(int)sensorY[i],2,2);
        if(sensorOn[i])circle((int)sensorX[i],(int)sensorY[i],3);
    }
}

/* Local lines rotate with the rover, including its small R badge. */
void roverLine(double f1,double l1,double f2,double l2,int color)
{
    setcolor(color);
    line((int)(robotX+f1*cos(heading)+l1*sin(heading)),
         (int)(robotY+f1*sin(heading)-l1*cos(heading)),
         (int)(robotX+f2*cos(heading)+l2*sin(heading)),
         (int)(robotY+f2*sin(heading)-l2*cos(heading)));
}

void drawRobot(int idle)
{
    int light;
    double lampX,lampY;
    /* Four separate tires with silver hubs, two on each side. */
    robotBox(-9,6,-4,10,DARKGRAY);
    robotBox(1,6,6,10,DARKGRAY);
    robotBox(-9,-10,-4,-6,DARKGRAY);
    robotBox(1,-10,6,-6,DARKGRAY);
    roverLine(-7,8,-5,8,WHITE);
    roverLine(3,8,5,8,WHITE);
    roverLine(-7,-8,-5,-8,WHITE);
    roverLine(3,-8,5,-8,WHITE);
    robotBox(-10,-6,5,6,CYAN);
    robotBox(-7,-4,2,4,BLUE);
    /* White R insignia on the blue central equipment hatch. */
    roverLine(-5,2,-5,-2,WHITE);
    roverLine(-5,2,-2,2,WHITE);
    roverLine(-2,2,-2,0,WHITE);
    roverLine(-2,0,-5,0,WHITE);
    roverLine(-4,0,-1,-2,WHITE);
    /* Front sensor bar stays at the exact locations used by detection. */
    robotBox(4,-7,7,7,DARKGRAY);
    roverLine(8,0,14,0,YELLOW);
    roverLine(14,0,11,2,YELLOW);
    roverLine(14,0,11,-2,YELLOW);
    light=idle ? YELLOW : (strcmp(motion,"RECOVERING")==0 ? LIGHTRED : LIGHTGREEN);
    lampX=robotX-9*cos(heading);lampY=robotY-9*sin(heading);
    setcolor(light);setfillstyle(SOLID_FILL,light);
    fillellipse((int)lampX,(int)lampY,1,1);
    drawSensors();
}

char *turnName(void)
{
    double a;
    if(segment==0) return "START / STRAIGHT";
    a=wrapAngle(calculateAngle(segment)-calculateAngle(segment-1));
    if(fabs(a)<10*PI/180) return "STRAIGHT";
    if(fabs(a)>=70*PI/180) return a<0 ? "SHARP LEFT" : "SHARP RIGHT";
    return a<0 ? "LEFT TURN" : "RIGHT TURN";
}

void showPathAnalysis(void)
{
    char b[70];
    textAt(470,95,YELLOW,"PATH ANALYSIS");
    sprintf(b,"Points     %d",pointCount);textAt(470,115,WHITE,b);
    sprintf(b,"Segments   %d",pointCount-1);textAt(470,132,WHITE,b);
    sprintf(b,"Length %.4g u",pathLength/viewScale);textAt(470,149,WHITE,b);
    sprintf(b,"Turns      %d",turns);textAt(470,166,WHITE,b);
    sprintf(b,"Sharp      %d",sharpTurns);textAt(470,183,WHITE,b);
    textAt(470,207,LIGHTGREEN,"Start (0,0)");
    sprintf(b,"End X %ld",worldX[pointCount-1]);textAt(470,224,LIGHTRED,b);
    sprintf(b,"End Y %ld",worldY[pointCount-1]);textAt(470,241,LIGHTRED,b);
}

void liveText(int x,int y,int color,char *s)
{
    /* Clear only this eight-pixel-high value before replacing its text. */
    setfillstyle(SOLID_FILL,BLACK);bar(x,y,637,y+7);
    textAt(x,y,color,s);
}

void livePanel(int paused)
{
    char b[70];
    textAt(470,95,YELLOW,"ROBOT LIVE DATA");
    sprintf(b,"X %.5g",userX());liveText(470,116,WHITE,b);
    sprintf(b,"Y %.5g",userY());liveText(470,132,WHITE,b);
    sprintf(b,"Angle %.1f deg",-heading*180.0/PI);liveText(470,146,WHITE,b);
    sprintf(b,"Speed %.2f px/tick",speed);liveText(470,160,WHITE,b);
    sprintf(b,"Segment %d / %d",segment+1,pointCount-1);liveText(470,182,LIGHTCYAN,b);
    if(segment<pointCount-2) sprintf(b,"Next %d",segment+2);
    else strcpy(b,"FINAL SEGMENT");
    liveText(470,201,LIGHTCYAN,b);
    sprintf(b,"L sensor %s",sensorOn[0]?"ON":"OFF");liveText(470,232,sensorOn[0]?LIGHTGREEN:LIGHTRED,b);
    sprintf(b,"C sensor %s",sensorOn[1]?"ON":"OFF");liveText(470,252,sensorOn[1]?LIGHTGREEN:LIGHTRED,b);
    sprintf(b,"R sensor %s",sensorOn[2]?"ON":"OFF");liveText(470,272,sensorOn[2]?LIGHTGREEN:LIGHTRED,b);
    liveText(470,302,YELLOW,turnName());
    liveText(470,325,WHITE,paused?"PAUSED":motion);
    sprintf(b,"Complete %5.1f%%",completion());liveText(470,352,LIGHTGREEN,b);
    sprintf(b,"Moved %.4g u",travelled/viewScale);liveText(470,374,LIGHTGRAY,b);
    textAt(470,399,LIGHTGRAY,"+/- speed 0.5..4");
    setcolor(CYAN);rectangle(470,416,632,430);
    setfillstyle(SOLID_FILL,LIGHTGREEN);
    if(completion()>0)bar(472,418,472+(int)(158*completion()/100),428);
}

void drawScene(int preview, int paused)
{
    int i;
    screenTitle(preview ? "CUSTOM PATH PREVIEW / YOUR COORDINATES CREATE THIS TRACK" :
                        "VIRTUAL LINE -> SENSORS -> STEERING -> CUSTOM PATH COMPLETED");
    drawGrid();drawPath();
    setcolor(LIGHTBLUE);
    for(i=1;i<trailCount;++i)
        line((int)trailX[i-1],(int)trailY[i-1],(int)trailX[i],(int)trailY[i]);
    drawPoints();
    /* Live rover is drawn only AFTER its clean background is captured. */
    if(preview)drawRobot(1);
    setcolor(DARKGRAY);line(459,84,459,430);
    if(preview) showPathAnalysis(); else livePanel(paused);
    textAt(16,448,WHITE,preview ? "CUSTOM PATH READY - ENTER starts / ESC returns to menu" :
                                        "P pause | R resume | S stop | Q menu | +/- speed");
}

int captureRoverBackground(void)
{
    backgroundX=(int)robotX-16;backgroundY=(int)robotY-16;
    if(imagesize(backgroundX,backgroundY,backgroundX+32,backgroundY+32)
       >sizeof(roverBackground))return 0;
    getimage(backgroundX,backgroundY,backgroundX+32,backgroundY+32,
             roverBackground);
    backgroundValid=1;
    return 1;
}

void updateRoverDrawing(double oldX,double oldY,int paused)
{
    int i, reached;
    /* Restore grid, track, labels and previous trail in one small copy. */
    if(backgroundValid)putimage(backgroundX,backgroundY,roverBackground,COPY_PUT);
    if(calculateDistance(oldX,oldY,robotX,robotY)>0.00001) {
        setcolor(LIGHTBLUE);
        line((int)oldX,(int)oldY,(int)robotX,(int)robotY);
    }
    reached=finished ? pointCount-1 : segment;
    setcolor(LIGHTGREEN);setfillstyle(SOLID_FILL,LIGHTGREEN);
    for(i=0;i<=reached;++i)
        fillellipse((int)xPoints[i],(int)yPoints[i],3,3);
    captureRoverBackground();
    drawRobot(paused);
}

void rememberTrail(void)
{
    int i;
    if(trailCount>0 && calculateDistance(robotX,robotY,
                           trailX[trailCount-1],trailY[trailCount-1])<3.0) return;
    if(trailCount==800) {
        for(i=1;i<400;++i) {trailX[i]=trailX[2*i];trailY[i]=trailY[2*i];}
        trailCount=400;
    }
    trailX[trailCount]=robotX;trailY[trailCount++]=robotY;
}

void showFinalReport(char *status)
{
    char b[100];
    screenTitle(finished ? "MISSION COMPLETED / YOUR CUSTOM PATH WAS FOLLOWED" : "SIMULATION ENDED");
    textAt(90,110,YELLOW,"SIMULATION REPORT");
    sprintf(b,"Status           : %s",status);textAt(90,150,WHITE,b);
    sprintf(b,"Path completion  : %.1f%%",completion());textAt(90,175,LIGHTGREEN,b);
    sprintf(b,"Points / segments: %d / %d",pointCount,pointCount-1);textAt(90,200,WHITE,b);
    sprintf(b,"Track length     : %.6g units",pathLength/viewScale);textAt(90,225,WHITE,b);
    sprintf(b,"Actual distance  : %.6g units",travelled/viewScale);textAt(90,250,WHITE,b);
    sprintf(b,"Turns / sharp    : %d / %d",turns,sharpTurns);textAt(90,275,WHITE,b);
    sprintf(b,"Final position   : (%.9g, %.9g)",userX(),userY());textAt(90,300,LIGHTCYAN,b);
    textAt(90,365,LIGHTGRAY,"Press any key to return to the menu.");
    getch();
}

void simulateRobot(void)
{
    int c, paused=0, stop=0, count;
    double oldX,oldY;
    char b[30];
    trailCount=0;frameCount=0;resetRobot();rememberTrail();drawScene(1,0);
    do {c=getch();if(c==27)return;} while(c!=13);
    for(count=3;count>=0;--count) {
        setfillstyle(SOLID_FILL,BLACK);bar(165,235,320,270);
        if(count) sprintf(b,"ROBOT READY %d",count);else strcpy(b,"GO!");
        textAt(175,247,YELLOW,b);delay(600);
    }
    /* One complete draw at the start; never clear the screen in the loop. */
    drawScene(0,0);backgroundValid=0;
    if(!captureRoverBackground()) {
        textAt(30,435,LIGHTRED,"Graphics image buffer unavailable. Press a key.");
        getch();return;
    }
    drawRobot(0);
    while(!finished && !stop) {
        if(kbhit()) {
            c=getch();
            if(c==0 || c==224) {getch();continue;}
            if(c=='p'||c=='P')paused=1;
            if(c=='r'||c=='R')paused=0;
            if(c=='s'||c=='S'||c=='q'||c=='Q')stop=1;
            if(c=='+'||c=='=')speed=clamp(speed+0.25,0.5,4.0);
            if(c=='-')speed=clamp(speed-0.25,0.5,4.0);
        }
        oldX=robotX;oldY=robotY;
        if(!paused && !stop) {updateRobot();rememberTrail();}
        updateRoverDrawing(oldX,oldY,paused);
        if(frameCount++%5==0 || finished || stop)livePanel(paused);
        delay(20);
    }
    showFinalReport(finished?"COMPLETED":"STOPPED BY USER");
}

void showInstructions(void)
{
    screenTitle("HOW IT WORKS / VIVA GUIDE");
    textAt(25,110,WHITE,"1. Choose 2..30 total points. P1 starts at (0,0).");
    textAt(25,140,WHITE,"2. X increases right; Y increases up. Auto-fit view.");
    textAt(25,170,WHITE,"3. Preview shows your coordinates, arrows and path analysis.");
    textAt(25,200,WHITE,"4. ENTER starts. L/C/R measure distance to the virtual line.");
    textAt(25,230,WHITE,"5. Green sensor = line detected; red = no detection.");
    textAt(25,260,WHITE,"6. Waypoints guide turns; sensor readings correct steering.");
    textAt(25,290,WHITE,"7. At corners the robot can rotate in place; no teleporting.");
    textAt(25,320,WHITE,"8. P pause, R resume, S stop, Q menu, +/- speed.");
    textAt(25,360,LIGHTCYAN,"Whole numbers, up to 9 digits; negative values allowed.");
    textAt(25,405,YELLOW,"Press any key for the menu.");getch();
}

int showMainMenu(void)
{
    int c;
    screenTitle("USER INPUT -> CUSTOM TRACK -> VIRTUAL SENSORS -> ROBOT STEERING");
    setcolor(CYAN);rectangle(100,110,540,395);
    textAt(150,140,LIGHTCYAN,"CREATE THE ROUTE. WATCH IT FOLLOW.");
    textAt(160,195,WHITE,"1. Create Custom Path");
    textAt(160,235,WHITE,"2. Run Sample Path");
    textAt(160,275,WHITE,"3. Instructions");
    textAt(160,315,WHITE,"4. Exit");
    textAt(120,370,LIGHTGRAY,"Software simulation / classic Turbo C BGI");
    do {c=getch();}while(c<'1'||c>'4');
    return c-'0';
}

int initGraphics(char *directory)
{
    int gd=VGA,gm=VGAHI,error;
    initgraph(&gd,&gm,directory);
    error=graphresult();
    if(error!=grOk) {
        printf("Graphics error: %s\nBGI directory: %s\n",grapherrormsg(error),directory);
        printf("Install EGAVGA.BGI and correct BGI_DIR or command-line path.\n");
        return 0;
    }
    settextstyle(DEFAULT_FONT,HORIZ_DIR,1);
    return 1;
}

int main(int argc,char *argv[])
{
    int choice;
    if(!initGraphics(argc>1?argv[1]:BGI_DIR))return 1;
    do {
        choice=showMainMenu();
        if(choice==1) {if(getCoordinates()) {analyzePath();simulateRobot();}}
        if(choice==2) {samplePath();analyzePath();simulateRobot();}
        if(choice==3)showInstructions();
    }while(choice!=4);
    closegraph();return 0;
}
#else
/* Reproducible controller checks, compiled as ANSI C on the host. */
int main(void)
{
    int test,i,j,ticks,total=0,oldSegment;
    double oldX,oldY,moved,maxStep;
    srand(17);
    for(test=0;test<1007;++test) {
        pointCount=2;
        if(test==0)samplePath();
        else if(test==1) {
            pointCount=4;
            xPoints[0]=50;yPoints[0]=200;xPoints[1]=400;yPoints[1]=200;
            xPoints[2]=51;yPoints[2]=201;xPoints[3]=399;yPoints[3]=202;
        } else if(test==2) {
            pointCount=30;
            for(i=0;i<30;++i){xPoints[i]=100+i;yPoints[i]=200;}
        } else {
            pointCount=2+rand()%29;
            for(i=0;i<pointCount;++i) {
                do {
                    j=0;
                    xPoints[i]=XMIN+rand()%(XMAX-XMIN+1);
                    yPoints[i]=YMIN+rand()%(YMAX-YMIN+1);
                    if(validateCoordinates(xPoints[i],yPoints[i],i)!=1)j=1;
                }while(j);
            }
        }
        speed=0.5+(test%15)*0.25;
        analyzePath();resetRobot();
        for(ticks=0;ticks<200000 && !finished;++ticks) {
            oldX=robotX;oldY=robotY;oldSegment=segment;
            updateRobot();
            moved=calculateDistance(oldX,oldY,robotX,robotY);
            maxStep=speed+0.000001;
            if(moved>maxStep || segment<oldSegment || segment>oldSegment+1) {
                printf("FAIL: nonincremental movement test %d\n",test);return 1;
            }
            if(segment>oldSegment && calculateDistance(robotX,robotY,
                                  xPoints[segment],yPoints[segment])>0.00002) {
                printf("FAIL: skipped waypoint\n");return 1;
            }
        }
        if(!finished || fabs(completion()-100)>0.00001 ||
             calculateDistance(robotX,robotY,xPoints[pointCount-1],
                                yPoints[pointCount-1])>0.00002) {
            printf("FAIL: completion test %d ticks %d\n",test,ticks);return 1;
        }
        total+=ticks;
    }
    pointCount=2;xPoints[0]=100;yPoints[0]=200;xPoints[1]=400;yPoints[1]=200;
    robotX=150;robotY=200;heading=0;detectSensors();
    if(sensorOn[0] || !sensorOn[1] || sensorOn[2])return 2;
    robotY=205;detectSensors();
    if(!sensorOn[0] || sensorOn[1] || sensorOn[2])return 3;
    robotY=195;detectSensors();
    if(sensorOn[0] || sensorOn[1] || !sensorOn[2])return 4;
    robotY=240;detectSensors();
    if(sensorOn[0] || sensorOn[1] || sensorOn[2])return 5;
    if(validateCoordinates(20,200,1)!=0 || validateCoordinates(100,200,1)!=-1)return 6;
    /* Prove detected side sensors actually change the steering command. */
    segment=finished=0;speed=2;robotX=150;robotY=205;heading=0;
    updateRobot();
    if(heading>=-0.07)return 7;
    robotX=150;robotY=195;heading=0;updateRobot();
    if(heading<=0.07)return 8;
    robotX=150;robotY=240;heading=PI;updateRobot();
    if(strcmp(motion,"TURNING") || heading==PI)return 9;
    for(test=0;test<5;++test) {
        pointCount=4;worldX[0]=worldY[0]=0;
        worldX[1]=test==0 ? 1 : 1000;
        worldY[1]=test==1 ? -2000 : 0;
        worldX[2]=test==2 ? -999999999L : 4000;
        worldY[2]=test==3 ? 999999999L : 5000;
        worldX[3]=test==4 ? -999999999L : 6000;
        worldY[3]=test==4 ? -999999999L : -7000;
        fitPath();analyzePath();resetRobot();speed=2;
        if(fabs(userX())>0.0001 || fabs(userY())>0.0001)return 10;
        for(i=0;i<pointCount;++i)
            if(xPoints[i]<XMIN-0.00001 || xPoints[i]>XMAX+0.00001 ||
               yPoints[i]<YMIN-0.00001 || yPoints[i]>YMAX+0.00001)return 11;
        for(ticks=0;ticks<200000 && !finished;++ticks)updateRobot();
        if(!finished || fabs(userX()-worldX[3])>0.001 ||
                      fabs(userY()-worldY[3])>0.001)return 12;
    }
    printf("PASS: 1007 paths, %d ticks; waypoint order, bounded motion, final position, sensors, validation.\n",total);
    printf("PASS: 5 auto-fit routes; zero origin, negative/large coordinates, bounds and world-space finish.\n");
    return 0;
}
#endif
