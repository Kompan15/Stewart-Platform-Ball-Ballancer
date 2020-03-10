
#include <Servo.h>

#include <Wire.h>

//MIN/MAX serwomechanizmów.

#define MAX 2200

#define MIN 800

//inicjalizacja JOYA
const int SW_pin = 2; // digital pin connected to switch output
const int X_pin = 0; // analog pin connected to X output
const int Y_pin = 1; // analog pin connected to Y output


//odbicia lustrzane serw.

#define INV1 1

#define INV2 3

#define INV3 5

//stałe przydatne

#define pi  3.14159

#define deg2rad 180/pi

#define deg30 pi/6

//tablica serw
Servo servo[6];

//pozycje zerowe serw
static int zero[6]={1500,1500,1460,1460,1500,1500};

//tablica przechowująca pozycję platformy
static float arr[6]={0,0.0,0, radians(0),radians(0),radians(0)};

//Actual degree of rotation of all servo arms, they start at 0 - horizontal, used to reduce
//complexity of calculating new degree of rotation
static float theta_a[6]={0.0,0.0,0.0, 0.0,0.0,0.0};

//Tablica aktualnych pozycji każdego z serw.
static int servo_pos[6];

//rotacje ramion serwomachanizmów w stosunku do osi x
const float beta[] = {-pi/2,pi/2,5*pi/6, -pi/6,pi/6,-5*pi/6},

//maximum servo positions, 0 is horizontal position
      servo_min=radians(-80),servo_max=radians(80),

//servo_mult - mnożnik konwertujący radiany->puls dla zadanego kąta.

//L1-długość ramienia serwa, L2 - długość popychacza + Snapy w [mm]

//z_home - height of platform above base, 0 is height of servo arms

servo_mult=400/(pi/4),L1 = 16,L2 = 155, z_home = 137;

//RD distance from center of platform to attachment points (arm attachment point)

//PD distance from center of base to center of servo rotation points (servo axis)

//theta_p-angle between two servo axis points, theta_r - between platform attachment points

//theta_angle-helper variable

//p[][]=x y values for servo rotation points

//re[]{}=x y z values of platform attachment points positions

//equations used for p and re will affect postion of X axis, they can be changed to achieve

//specific X axis position

const float RD = 114, PD = 105, theta_p = radians(50),

theta_angle=(pi/3-theta_p)/2, theta_r = radians(20),

      p[2][6]={

          {
            -PD*cos(deg30-theta_angle),-PD*cos(deg30-theta_angle),

            PD*sin(theta_angle),PD*cos(deg30+theta_angle),

            PD*cos(deg30+theta_angle),PD*sin(theta_angle)
         },

         {
            -PD*sin(deg30-theta_angle),PD*sin(deg30-theta_angle),

            PD*cos(theta_angle),PD*sin(deg30+theta_angle),

            -PD*sin(deg30+theta_angle),-PD*cos(theta_angle)
         }

      },

      re[3][6] = {

          {

              -RD*sin(deg30+theta_r/2),-RD*sin(deg30+theta_r/2),

              -RD*sin(deg30-theta_r/2),RD*cos(theta_r/2),

              RD*cos(theta_r/2),-RD*sin(deg30-theta_r/2),

          },{

              -RD*cos(deg30+theta_r/2),RD*cos(deg30+theta_r/2),

              RD*cos(deg30-theta_r/2),RD*sin(theta_r/2),

              -RD*sin(theta_r/2),-RD*cos(deg30-theta_r/2),

          },{

              0,0,0,0,0,0

          }

};

//arrays used for servo rotation calculation

//H[]-center position of platform can be moved with respect to base, this is

//translation vector representing this move

static float M[3][3], rxp[3][6], T[3], H[3] = {0,0,z_home};

void setup(){

  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);

//attachment of servos to PWM digital pins of arduino

   servo[0].attach(3, MIN, MAX);

   servo[1].attach(5, MIN, MAX);

   servo[2].attach(6, MIN, MAX);

   servo[3].attach(9, MIN, MAX);

   servo[4].attach(10, MIN, MAX);

   servo[5].attach(11, MIN, MAX);

//begin of serial communication

   Serial.begin(9600);

//putting into base position

   setPos(arr);
   
}

//function calculating needed servo rotation value

float getAlpha(int *i){

   static int n;

   static float th=0;

   static float q[3], dl[3], dl2;

   double min=servo_min;

   double max=servo_max;

   n=0;

   th=theta_a[*i];

   while(n<20){

    //calculation of position of base attachment point (point on servo arm where is leg connected)

      q[0] = L1*cos(th)*cos(beta[*i]) + p[0][*i];

      q[1] = L1*cos(th)*sin(beta[*i]) + p[1][*i];

      q[2] = L1*sin(th);

    //calculation of distance between according platform attachment point and base attachment point

      dl[0] = rxp[0][*i] - q[0];

      dl[1] = rxp[1][*i] - q[1];

      dl[2] = rxp[2][*i] - q[2];

      dl2 = sqrt(dl[0]*dl[0] + dl[1]*dl[1] + dl[2]*dl[2]);

    //if this distance is the same as leg length, value of theta_a is corrent, we return it

      if(abs(L2-dl2)<0.01){

         return th;

      }

    //if not, we split the searched space in half, then try next value

      if(dl2<L2){

         max=th;

      }else{

         min=th;

      }

      n+=1;

      if(max==servo_min || min==servo_max){

         return th;

      }

      th = min+(max-min)/2;

   }

   return th;

}



//function calculating rotation matrix

void getmatrix(float pe[])

{

   float psi=pe[5];

   float theta=pe[4];

   float phi=pe[3];

   M[0][0] = cos(psi)*cos(theta);

   M[1][0] = -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);

   M[2][0] = sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);



   M[0][1] = sin(psi)*cos(theta);

   M[1][1] = cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);

   M[2][1] = cos(theta)*sin(phi);



   M[0][2] = -sin(theta);

   M[1][2] = -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);

   M[2][2] = cos(theta)*cos(phi);

}

//calculates wanted position of platform attachment poins using calculated rotation matrix

//and translation vector

void getrxp(float pe[])

{

   for(int i=0;i<6;i++){

      rxp[0][i] = T[0]+M[0][0]*(re[0][i])+M[0][1]*(re[1][i])+M[0][2]*(re[2][i]);

      rxp[1][i] = T[1]+M[1][0]*(re[0][i])+M[1][1]*(re[1][i])+M[1][2]*(re[2][i]);

      rxp[2][i] = T[2]+M[2][0]*(re[0][i])+M[2][1]*(re[1][i])+M[2][2]*(re[2][i]);

   }

}

//function calculating translation vector - desired move vector + home translation vector

void getT(float pe[])

{

   T[0] = pe[0]+H[0];

   T[1] = pe[1]+H[1];

   T[2] = pe[2]+H[2];

}



unsigned char setPos(float pe[]){

    unsigned char errorcount;

    errorcount=0;

    for(int i = 0; i < 6; i++)

    {

        getT(pe);

        getmatrix(pe);

        getrxp(pe);

        theta_a[i]=getAlpha(&i);
        
        if(i==INV1||i==INV2||i==INV3){

            servo_pos[i] = constrain(zero[i] - (theta_a[i])*servo_mult, MIN,MAX);

        }

        else{

            servo_pos[i] = constrain(zero[i] + (theta_a[i])*servo_mult, MIN,MAX);

        }

    }



    for(int i = 0; i < 6; i++)

    {

        if(theta_a[i]==servo_min||theta_a[i]==servo_max||servo_pos[i]==MIN||servo_pos[i]==MAX){

            errorcount++;

        }

        servo[i].writeMicroseconds(servo_pos[i]);

    }

    return errorcount;

}

//main control loop, obtain requested action from serial connection, then execute it

void loop()

{
arr[0] = 0;
arr[1] = 0;
arr[2] = 0;
arr[3] = (analogRead(X_pin)-506)*(0.12/506);
arr[4] = (analogRead(Y_pin)-506)*(0.12/506);
arr[5] = radians(0);
setPos(arr);
//action to change position of platform, obtain 6 values representing desired position

}

void retPos(){

   for(int i=0;i<6;i++){

       long val;

       if(i<3){

           val=(long)(arr[i]*100*25.4);

       }else{

           val=(long)(arr[i]*100*deg2rad);

       }

       Serial.write(val);

       Serial.write((val>>8));

       Serial.write((val>>16));

       Serial.write((val>>24));

       Serial.flush();

   }

}
