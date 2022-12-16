#include<Servo.h>
Servo myservo1;  // Create a servo class
Servo myservo2;  // Create a servo class
Servo myservo3;  // Create a servo class

#define DEBUG 1
int delayTime;
// arm length
double L1 = 65; 
double L2 = 130;
double L3 = 65;
double L4 = sqrt((L2)*(L2)+(L3)*(L3));
//current position
double curx, cury, curz;
double pi = 3.141592654;

void setup()
{
myservo1.attach(9);  //Set the servo control pin as D9
myservo2.attach(6);  //Set the servo control pin as D6
myservo3.attach(5);  //Set the servo control pin as D5

#ifdef DEBUG
Serial.begin(9600);
#endif
}

void loop() {
// initial position
JointWrite(0,0);
JointWrite(1,0);
JointWrite(2,90);
delay(1000);
curx=130;
cury=curz=0;

// drawing
delayTime=30;

//GoToByJoint(0,80,180,30);
//DrawLine100();
DrawSquare50(50);
DrawTriangle50(50);

while(1); // to stop
}

void DrawTriangle50(double length)
{
double zheight=-10;
GoToXyz(130, 0, 0);
GoToXyz(130, 0, zheight);
GoToXyz(130, length, zheight);
GoToXyz(130, length+10, zheight);//to fix backlash
GoToXyz(130, length, zheight);
GoToXyz(130+length, length/2, zheight);
GoToXyz(130, 0, zheight);
GoToXyz(130,0,0);
}

void DrawSquare50(double length)
{
double zheight=-10;
GoToXyz(130, 0, 0);
GoToXyz(130, 0, zheight);
GoToXyz(130, length, zheight);
GoToXyz(130+length, length, zheight);
GoToXyz(130+length, 0, zheight);
GoToXyz(130, 0, zheight);
GoToXyz(130,0,0);
}

void DrawLine100()
{
double zheight=-10;
GoToXyz(130, 0, 0);
GoToXyz(130, 0, zheight);
GoToXyz(130, 100, zheight);
GoToXyz(130, 0, zheight);
GoToXyz(130,0,0);
}

//Linear control by interpolation
void GoToXyz(double x, double y, double z)
{
double j0, j1, j2;
double xi, yi,zi;
double distance = sqrt( (x-curx)*(x-curx)+(y-cury)*(y-cury)+(z-curz)*(z-curz));
int n;
int h = 1; // grid size
n = distance / h;
int i;
for(i = 0; i< n; i++)
   {
    // coordinates of each node
    xi = ((x-curx)/(double)n)*(double)i+curx;   
    yi = ((y-cury)/(double)n)*(double)i+cury;
    zi = ((z-curz)/(double)n)*(double)i+curz;

  // To convert coordinates into angles of each nodes
  j0 = FindTh0(xi,yi,zi);
  j1 = Cord1Th1(xi,yi,zi);
  j2 = Cord2Th2(xi,yi,zi);

  // Function to move to each node
   JointWrite(0,RtoD(j0));
   JointWrite(1,RtoD(j1));
   JointWrite(2,RtoD(j2));
   delay(delayTime);
   //DebugXyzRadian(i,xi,yi,zi,j0,j1,j2);
   }
 
 DebugNameRadian("j0", j0);
 DebugNameRadian("j1", j1);
 DebugNameRadian("j2", j2);
  Serial.print("distnace is ");
   Serial.println(distance);;

 // to supplement the compensated rounding errors   
   j0 = FindTh0(x,y,z);
   j1 = Cord1Th1(x,y,z);
   j2 = Cord2Th2(x,y,z);
   JointWrite(0,RtoD(j0));
   JointWrite(1,RtoD(j1));
   JointWrite(2,RtoD(j2));
   delay(delayTime);

   //DebugXyzRadian(n,x,y,z,j0,j1,j2);

   //To update curx, cury, curz
   curx = x;
   cury = y;
   curz = z;
}

double Cord1Th1(double x, double y, double z)
{
  double theta3 = FindTh3(x,y,z);
  double theta4 = FindTh4(x,y,z);
  double theta5 = FindTh5(theta3,theta4);
  double theta6 = FindTh6(x,y,z);
  double theta1 = FindTh1(theta5,theta6);
  return theta1;
}

double Cord2Th2(double x, double y, double z)
{
  double theta3 = FindTh3(x,y,z);
  double theta7 = FindTh7(L2,L3);
  double theta2 = FindTh2(theta3,theta7);
  return theta2;
}

double FindTh0(double x, double y, double z)
{
  double th0 = atan(y/x);
  return th0;
}

double FindTh3(double x,double y,double z)
{ 
 double dd= x*x+y*y+z*z;
 double L1_4= L1*L1 + L4*L4;
 double L1Times4= 2*L1*L4;
 double th3_Cal= (L1_4-dd)/L1Times4;
 double th3a = acos(th3_Cal);
 return th3a;
}
double FindTh4(double x,double y,double z)
{
  double d = sqrt(x*x +y*y+z*z);
  double th4 = acos((x*x+y*y+z*z+L4*L4-L1*L1)/(2*L4*d));
  return th4;
}
double FindTh5(double th3, double th4)
{
  double th5 = pi-th3-th4;
  return th5;
}

double FindTh6(double x, double y, double z)
{
  double d2 = sqrt(x*x+y*y);
  double th6 = atan(z/d2);
  return th6;
}

double FindTh1(double th5, double th6)
{
  double th1 = pi/2 -th5-th6;
  return th1;
}

double FindTh7(double L2, double L3)
{
  double th7 = atan(L3/L2);
  return th7;
}

double FindTh2(double th3, double th7)
{
  double th2 = pi -th3-th7;
  return th2;
}







double RtoD(double Radian)
{
  double D = Radian *(180/pi);
  return D;
}

double DtoR(double Degree)
{
  double R = Degree *(pi/180);
  return R;
}

void DebugNameRadian(String radianName, double radian1)
{
  Serial.print("This is ");
  Serial.print(radianName);
  Serial.print(" = ");
  Serial.print(radian1);

  Serial.print(" (rad), angle = ");
  Serial.print(RtoD(radian1)); 
  Serial.print(" (deg)\n");
}

void DebugXyzRadian(int index, double x, double y, double z, double j0, double j1, double j2)
{
  Serial.print(index);
  Serial.print(", xyz( ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.print(" ) joint( ");
  Serial.print(RtoD(j0));
  Serial.print(", ");
  Serial.print(RtoD(j1));
  Serial.print(", ");
  Serial.print(RtoD(j2));
  Serial.print(" )\n");
}

void GoToByJoint(int in_jointindex, int start, int end, int in_delay)
{
  int n;
  int i;
  if(start<end)
  {
    n = end - start+1;
    for (i=0; i<n;i++)
    {
      PulseWrite(in_jointindex, start+i);
      delay(in_delay);
    }
  }
else
  {
    n = start - end+1;
    for (i=0; i<n; i++)
    {
      PulseWrite(in_jointindex, start-i);
      delay(in_delay);
    }
  }
}

void PulseWrite(int in_jointindex, double in_pulse)
{
  if(in_jointindex==0)
  {
    myservo1.write(in_pulse);
  }
  else if(in_jointindex==1)
  {
    myservo2.write(in_pulse);
  }
  else if(in_jointindex==2)
  {
    myservo3.write(in_pulse);
  }
}

void JointWrite(int in_jointindex, double in_angle)
{
  double machineA;
  if(in_jointindex==0)
  {
    machineA = in_angle * (180.0-80.0)/90.0 + 80.0;
    myservo1.write(machineA);
    //Serial.println(machineA);
  }
  else if(in_jointindex==1)
  {
    machineA = in_angle * (150.0-50.0)/90.0 + 50.0;
    myservo2.write(machineA);
    //Serial.println(machineA);
  }
  else if(in_jointindex==2)
  {
    machineA = in_angle * (35.0-135.0)/90.0 + 135.0;
    myservo3.write(machineA);
    //Serial.println(machineA);
  }
}

