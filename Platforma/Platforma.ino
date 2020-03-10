#include <Servo.h>
#include <Wire.h>
//MIN/MAX serwomechanizmów.
#define MAX 2200
#define MIN 800
//definicja pinow ekranu
#define PIN_TR A1
#define PIN_BR A2
#define PIN_S  A3
#define PIN_BL A4
#define PIN_TL A5

//setup diody RGB i przycisku przerwania.

const int buttonPin = 13;     // the number of the pushbutton pin
const int red = 7;
const int green = 8;
const int blue = 12;
int stanPrzycisku = 0; 
//definicje zmiennych
int Xi = 1, Yi = 1, xDi = 0, yDi = 0; //definicje zmiennych dla ekranu i filtracji.
float X, Y, X1, Y1; //zmienne przechowujące ostateczne współrzędne, już po wygładzeniu.
float Catch_Initial_X, Catch_Initial_Y, Catch_X_UBound, Catch_Y_UBound, Catch_X_DBound, Catch_Y_DBound;
int Ilosc_Probek = 2; //ilosc probek na kazdej osi
float xSu[3], ySu[3], xSum, ySum; //xSum i ySum podzielone przez ilość próbek dają w ostateczności przyzwoicie wygładzony sygnał.
float dX, dY;
float Dx, Dy;
float Kp = 0.0002850, Kd = 0.0018, Ki = 0.0000025; //wspolczynniki do kalibracji PID
float Catch_X, Catch_Y, Catch_dX, Catch_dY, Catch_iX, Catch_iY, cac,cdc; //PID Out terms.
int xTemp, yTemp; //Temporary terms to help calculate median.

  //prptotypując...
  float czasXmax = 0, czasYmax=0;
  unsigned long time0,time1;

 //smienne symboliczne bledu.
  float Xa,Ya;
  float eX,eY,eX1,eY1;
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
static int zero[6] = {1540, 1460, 1540, 1500, 1500, 1500};
//tablica przechowująca pozycję platformy
static float arr[6] = {0, 0.0, 0, radians(0), radians(0), radians(0)};
//Actual degree of rotation of all servo arms, they start at 0 - horizontal, used to reduce
//complexity of calculating new degree of rotation
static float theta_a[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//Tablica aktualnych pozycji każdego z serw.
static int servo_pos[6];
//rotacje ramion serwomachanizmów w stosunku do osi x
const float beta[] = { -pi / 2, pi / 2, 5 * pi / 6, -pi / 6, pi / 6, -5 * pi / 6},
                     //maksymalne dozwolone ruchy serw - bezpieczenstwo przede wszystkim.
                     servo_min = radians(-70), servo_max = radians(70),
                     //servo_mult - mnożnik konwertujący radiany->puls dla zadanego kąta.
                     //L1-długość ramienia serwa, L2 - długość popychacza + Snapy w [mm]
                     //z_home - height of platform above base, 0 is height of servo arms
                     servo_mult = 400 / (pi / 4), L1 = 16, L2 = 155, z_home = 150;
//RD - odległość od środka platformy do snapów
//PD  - odleglosc od srodka podstawy do miejsc zaczepienia orczykow (osie serw)
//theta_p- kat pomiedzy wektorami wskazujacymi polozenie punktow obrotu orczykow, theta_r - kat pomiedzy punktami zaczepienia snapow na platformie
//theta_angle- zmienna pomocnicza
//p[][]=x y koordynanty punktow rotacji
//re[]{}=x y z koordynanty snapow platformy
//equations used for p and re will affect postion of X axis, they can be changed to achieve
//specific X axis position
const float RD = 102, PD = 105, theta_p = radians(50),
            theta_angle = (pi / 3 - theta_p) / 2, theta_r = radians(8),
p[2][6] = {
  {
    -PD * cos(deg30 - theta_angle), -PD * cos(deg30 - theta_angle),
    PD * sin(theta_angle), PD * cos(deg30 + theta_angle),
    PD * cos(deg30 + theta_angle), PD * sin(theta_angle)
  },
  {
    -PD * sin(deg30 - theta_angle), PD * sin(deg30 - theta_angle),
    PD * cos(theta_angle), PD * sin(deg30 + theta_angle),
    -PD * sin(deg30 + theta_angle), -PD * cos(theta_angle)
  }
},
re[3][6] = {
  {
    -RD * sin(deg30 + theta_r / 2), -RD * sin(deg30 + theta_r / 2),
    -RD * sin(deg30 - theta_r / 2), RD * cos(theta_r / 2),
    RD * cos(theta_r / 2), -RD * sin(deg30 - theta_r / 2),
  }, {
    -RD * cos(deg30 + theta_r / 2), RD * cos(deg30 + theta_r / 2),
    RD * cos(deg30 - theta_r / 2), RD * sin(theta_r / 2),
    -RD * sin(theta_r / 2), -RD * cos(deg30 - theta_r / 2),
  }, {
    0, 0, 0, 0, 0, 0
  }
};
//tablice używane do rotacji.
//H[]-wektor translacji z podstawy do platformy.
static float M[3][3], rxp[3][6], T[3], H[3] = {0, 0, z_home};
void Koordynanty(float &a, float &b) {
  //zgodnie z algorytmem działania, ustawiam odpowiednie piny na stan wysoki - patrz do dziennika pokładowego (zielona ramka)
  //Pomiar X:
  digitalWrite(PIN_TR, LOW);
  digitalWrite(PIN_BR, LOW);
  digitalWrite(PIN_TL, HIGH);
  digitalWrite(PIN_BL, HIGH);
  //wygładzamX
  delay(2);
  //filtruj mediana
  a = analogRead(PIN_S);
  xSum = 0;
  Xi = 0;
  //POMIAR Y
  digitalWrite(PIN_TR, HIGH);
  digitalWrite(PIN_TL, HIGH);
  digitalWrite(PIN_BL, LOW);
  digitalWrite(PIN_BR, LOW);
  delay(1);
  //wygładzamY
    delay(2);
  //filtruj mediana
  b = analogRead(PIN_S);
  ySum = 0;
  Yi = 0;
}
void setup() {
  //Ustawienie pinów obsługi ekranu.
  pinMode(PIN_TR, OUTPUT);
  pinMode(PIN_TL, OUTPUT);
  pinMode(PIN_BL, OUTPUT);
  pinMode(PIN_BR, OUTPUT);
  digitalWrite(PIN_TR, LOW);
  digitalWrite(PIN_TL, LOW);
  digitalWrite(PIN_BL, LOW);
  digitalWrite(PIN_BR, LOW);
  pinMode(PIN_S, INPUT);
  //Przyporządkowanie pinów do poszczególnych serv.
  servo[0].attach(3, MIN, MAX);
  servo[1].attach(5, MIN, MAX);
  servo[2].attach(6, MIN, MAX);
  servo[3].attach(9, MIN, MAX);
  servo[4].attach(10, MIN, MAX);
  servo[5].attach(11, MIN, MAX);
  //rozpocznij komunikację
 //inicjalizacja pinów RGB i przycisku.
  pinMode(buttonPin, INPUT);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  Serial.begin(9600);
  //wysteruj pozycję początkową
  Ustaw_Pozycje(arr);
  arr[0] = 0;
  arr[1] = 0;
  arr[2] = 15;
  arr[3] = radians(0);
  arr[4] = radians(0);
  arr[5] = radians(0);
  Ustaw_Pozycje(arr); //zmien pozycje
  /*inicjalizacja punktów stabilnosci, NIE USTAWIAC GO BLIZEJ NIZ W POLOWIE ODLEGLOSCI OD SRODKA DO KRAWEDZI
    - dalej algorytm po prostu nie jest na tyle ogarniety.*/
  Koordynanty(Catch_Initial_X, Catch_Initial_Y);
  Koordynanty(Catch_Initial_X, Catch_Initial_Y);
  Koordynanty(Catch_Initial_X, Catch_Initial_Y);
  arr[0] = 0;
  arr[1] = 0;
  arr[2] = 5;
  arr[3] = radians(0);
  arr[4] = radians(0);
  arr[5] = radians(0);
  Ustaw_Pozycje(arr); //zmien pozycje
  kalibracja(Catch_X_DBound, Catch_Y_DBound, Catch_X_UBound, Catch_Y_UBound);
  arr[0] = 0;
  arr[1] = 0;
  arr[2] = 0;
  arr[3] = radians(0);
  arr[4] = radians(0);
  arr[5] = radians(0);
  Ustaw_Pozycje(arr); //zmien pozycje
}
void kalibracja(float &xDBoundary, float &yDBoundary, float &xUBoundary, float &yUBoundary) {
  float xMIN = 500, yMIN = 500, yMAX = 0, xMAX = 0;
  while (millis() < 10000) {
    digitalWrite(blue,HIGH);
    digitalWrite(green,LOW);
    digitalWrite(red,LOW);
    Koordynanty(X, Y);
    X += -Catch_Initial_X;
    Y += -Catch_Initial_Y;
    if (X < xMIN) {
      xMIN = X;
    }
    if (X > xMAX) {
      xMAX = X;
    }
    if (Y < yMIN) {
      yMIN = Y;
    }
    if (Y > yMAX) {
      yMAX = Y;
    }
  }
  //przechwycenie granic układu, lepsza praca...
  xDBoundary = xMIN;
  yDBoundary = yMIN;
  yUBoundary = yMAX;
  xUBoundary = xMAX;
}
//funkcja wyliczania żądanego kąta na serwomechanizmie
float getAlpha(int *i) {
  static int n;
  static float th = 0;
  static float q[3], dl[3], dl2;
  double min = servo_min;
  double max = servo_max;
  n = 0;
  th = theta_a[*i];
  while (n < 20) {
    //obliczenia współrzędnych połączenia pomiędzy orczykami a popychaczami.
    q[0] = L1 * cos(th) * cos(beta[*i]) + p[0][*i]; //p[][]=x y koordynanty punktow rotacji
    q[1] = L1 * cos(th) * sin(beta[*i]) + p[1][*i];
    q[2] = L1 * sin(th);
    //obliczenie dystansu pomiędzy punktami zaczepienia platformy a punktami zaczepienia podstawy
    dl[0] = rxp[0][*i] - q[0];
    dl[1] = rxp[1][*i] - q[1];
    dl[2] = rxp[2][*i] - q[2];
    dl2 = sqrt(dl[0] * dl[0] + dl[1] * dl[1] + dl[2] * dl[2]);
    //jeżeli dystans d12 jest równy długości popychacza obliczenia są poprawne i można liczyć dalej.
    if (abs(L2 - dl2) < 0.01) {
      return th;
    }
    //if not, we split the searched space in half, then try next value
    if (dl2 < L2) {
      max = th;
    } else {
      min = th;
    }
    n += 1;
    if (max == servo_min || min == servo_max) {
      return th;
    }
    th = min + (max - min) / 2;
  }
  return th;
}
//funkcja obliczająca macierz rotacji
void Licz_Macierz_Rotacji(float pe[])
{
  //kąty przekazane przez użytkownika.
  float psi = pe[5];
  float theta = pe[4];
  float phi = pe[3];
  M[0][0] = cos(psi) * cos(theta);                           //OK pierwsza kolumna pierwszy element
  M[1][0] = -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi); //OK druga kolumna pierwszy element
  M[2][0] = sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(theta); //OK trzecia kolumna pierwszy element
  M[0][1] = sin(psi) * cos(theta);                           //OK pierwsza kolumna drugi element
  M[1][1] = cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi); //OK druga kolumna drugi element
  M[2][1] = -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi); //OK druga kolumna, trzeci element
  M[0][2] = -sin(theta);                                     //OK pierwsza kolumna trzeci element
  M[1][2] =  cos(theta) * sin(phi);                          //OK druga kolumna trzeci element
  M[2][2] = cos(theta) * cos(phi);                           //OK trzecia kolumna trzeci element
}
void Licz_Nowe_Zaczepienie(float pe[])
{
  for (int i = 0; i < 6; i++) {
    rxp[0][i] = T[0] + M[0][0] * (re[0][i]) + M[0][1] * (re[1][i]) + M[0][2] * (re[2][i]); //re[]{}=x y z koordynanty snapow platformy, Tutaj rozwiązywane jest równanie T+M*(re
    rxp[1][i] = T[1] + M[1][0] * (re[0][i]) + M[1][1] * (re[1][i]) + M[1][2] * (re[2][i]);
    rxp[2][i] = T[2] + M[2][0] * (re[0][i]) + M[2][1] * (re[1][i]) + M[2][2] * (re[2][i]);
  }
}
//Oblicz wektor translacji
void Licz_Wektor_Translacji(float pe[])
{
  T[0] = pe[0] + H[0];
  T[1] = pe[1] + H[1];
  T[2] = pe[2] + H[2];
}
unsigned char Ustaw_Pozycje(float pe[]) {
  unsigned char errorcount;
  errorcount = 0;
  for (int i = 0; i < 6; i++)
  {
    Licz_Wektor_Translacji(pe);
    Licz_Macierz_Rotacji(pe);
    Licz_Nowe_Zaczepienie(pe);
    theta_a[i] = getAlpha(&i);
    if (i == INV1 || i == INV2 || i == INV3) { //Bierz pod uwage orientacje serwa.
      servo_pos[i] = constrain(zero[i] - (theta_a[i]) * servo_mult, MIN, MAX);
    }
    else {
      servo_pos[i] = constrain(zero[i] + (theta_a[i]) * servo_mult, MIN, MAX);
    }
  }
  for (int i = 0; i < 6; i++)
  {
    if (theta_a[i] == servo_min || theta_a[i] == servo_max || servo_pos[i] == MIN || servo_pos[i] == MAX) {
      errorcount++;
    }
    servo[i].writeMicroseconds(servo_pos[i]);
  }
  return errorcount;
}
//three functions calculating each P I D term.
void proporcjonalny(float &a, float &b) {
  a = constrain(Kp * eX,-0.08,0.08);
  b = constrain(Kp * eY,-0.08,0.08);}

void pochodna(float &da, float &db) {
  float deX = eX-eX1;
  float deY = eY-eY1;
  da = constrain(Kd * deX,-0.05,0.05);
  db = constrain(Kd * deY,-0.05,0.05);
  }
  
void calka(float &ca, float &cd) {
  float cX = Ki*eX;
  float cY = Ki*eY;
  cac += cX; //sumowanie bledu
  cdc += cY;
  ca = constrain(cac,-0.06,0.06); //wartosci sterujace sa saturowane dla bezpieczenstwa.
  cd = constrain(cdc,-0.06,0.06);}

void licz_szybkosc(float a,float b,unsigned long tt,float &CXmax,float &CYmax){
  float czasX = a*(0.0346/(Catch_X_UBound-Catch_X_DBound))/(tt*pow(10,-6)); //te stale to rozmiary ekranu.
  float czasY = b*(0.0197/(Catch_Y_UBound-Catch_Y_DBound))/(tt*pow(10,-6));
  if(czasX>CXmax){CXmax = czasX;}
  if(czasY>CYmax){CYmax = czasY;}
  Serial.print(CXmax);
  Serial.print("\t");
  Serial.write(",");

  Serial.print(CYmax);
  Serial.print("\t");
  Serial.write(",");
  }

  void wskaznik(float a, float b){

    if(abs(a)>0.02 || abs(b)>0.02){
      digitalWrite(red,HIGH);
      digitalWrite(green,LOW);
      digitalWrite(blue,LOW);} else 
    {
      digitalWrite(green,HIGH);
      digitalWrite(red,LOW);
      digitalWrite(blue,LOW);}
    
    }

  void licz_przyspieszenie(float a, float b){
    Xa = 9.81*sin(a*deg2rad);
    Ya = 9.81*sin(b*deg2rad);
    }

  void kalibruj_tymczasowy(float &a, float &b){
    a += -Catch_Initial_X;
    b += -Catch_Initial_Y;
    }
void loop()
{
  Koordynanty(X, Y); //zdobądź koordynanty
  time0 = micros(); //przechwyć czas
  kalibruj_tymczasowy(X,Y);
  eX = X; //przechwycenie do zmiennej symbolicznej 
  eY = Y;//przechwycenie do zmiennej symbolicznej 
  /*nastepuje wykonanie czlonow*/
  proporcjonalny(Catch_X, Catch_Y);
  pochodna(Catch_dX, Catch_dY);
  calka(Catch_iX, Catch_iY);
  X1 = X; Y1 = Y; //zbieram wartosc dla pochodnej.
  eX1 = X1; eY1 = Y1;//przechwycenie do zmiennej symbolicznej 
  time1 = micros()-time0;
  Serial.print(Catch_X);
  Serial.print("\t");
  Serial.write(",");
  Serial.print(Catch_Y);
  Serial.print("\t");
  Serial.write(",");
  Serial.print(Catch_dX);
  Serial.print("\t");
  Serial.write(",");
  Serial.print(Catch_dY);
  Serial.print("\t");
  Serial.write(",");
  Serial.print(Catch_iX);
  Serial.print("\t");
  Serial.write(",");
  Serial.println(Catch_iY);

  wskaznik(Catch_dY,Catch_dX);
  arr[0] = 0;
  arr[1] = 0;
  arr[2] = 0;
  arr[3] = (Catch_Y + Catch_dY + Catch_iY);
  arr[4] = (Catch_X + Catch_dX + Catch_iX);
  arr[5] = radians(0);
  Ustaw_Pozycje(arr); //zmien pozycje
}
void retPos() {
  for (int i = 0; i < 6; i++) {
    long val;
    if (i < 3) {
      val = (long)(arr[i] * 100 * 25.4);
    } else {
      val = (long)(arr[i] * 100 * deg2rad);
    }
    Serial.write(val);
    Serial.write((val >> 8));
    Serial.write((val >> 16));
    Serial.write((val >> 24));
    Serial.flush();
  }
}
