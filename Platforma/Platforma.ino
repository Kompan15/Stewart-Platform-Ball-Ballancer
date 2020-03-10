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
const int buttonPin = 13;
const int red = 7;
const int green = 8;
const int blue = 12;
int stanPrzycisku = 0; 
//definicje zmiennych
int Xi = 1, Yi = 1, xDi = 0, yDi = 0; //definicje zmiennych dla ekranu i filtracji.
int X, Y, X1, Y1; //zmienne przechowujące ostateczne współrzędne, już po wygładzeniu.
int Catch_Initial_X, Catch_Initial_Y, Catch_X_UBound, Catch_Y_UBound, Catch_X_DBound, Catch_Y_DBound;
int Ilosc_Probek = 4; //ilosc probek na kazdej osi
int xSu[5], ySu[5], xSum, ySum; //xSum i ySum podzielone przez ilość próbek dają w ostateczności przyzwoicie wygładzony sygnał.
float dX, dY;
float Dx, Dy;
float Kp = 0.0003, Kd = 0.002, Ki = 0.0000025; //wspolczynniki do kalibracji PID
float Catch_X, Catch_Y, Catch_dX, Catch_dY, Catch_iX, Catch_iY, cac,cdc; //Wyjście łańcucha sumacyjnego.
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
static int Neutralna[6] = {1540, 1460, 1540, 1500, 1500, 1500};
static float Pozycja[6] = {0, 0.0, 0, radians(0), radians(0), radians(0)}; //tablica przechowująca pozycję platformy
static float Kat_Orczyka[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //kąt domyslny, dodaje/odejmuje go, w zaleznosci od wyniku.
//Tablica aktualnych pozycji każdego z serw.
static int Puls_Na_Pozycji[6];
//rotacje ramion serwomachanizmów w stosunku do osi x
const float beta[] = { -pi / 2, pi / 2, 5 * pi / 6, -pi / 6, pi / 6, -5 * pi / 6}, //maksymalne dozwolone ruchy serw - bezpieczenstwo przede wszystkim.
Serwo_Minimum = radians(-70), Serwo_Maksimum = radians(70),
//Orczyk-długość ramienia serwa, Popychacz - długość popychacza + Snapy w [mm]
//H_Platformy - height of platform above base, 0 is height of servo arms
Mnoznik = 400 / (pi / 4), Orczyk = 16, Popychacz= 155, H_Platformy = 150; // mm
//Snap_platforma - odległość od środka platformy do snapów
//Koord_wal- odleglosc od srodka podstawy do miejsc zaczepienia orczykow (osie serw)
//Kat_Waly_Podstawy- kat pomiedzy wektorami wskazujacymi polozenie punktow obrotu orczykow, Kat_Snapy_Platformy - kat pomiedzy punktami zaczepienia snapow na platformie
//Kat_Konstrukcyjny- zmienna pomocnicza
//p[][]=x y Koordynaty punktow rotacji
//re[]{}=x y z Koordynaty snapow platformy
//equations used for p and re will affect postion of X axis, they can be changed to achieve
//specific X axis position
const float Snap_platforma = 102, Koord_wal = 105, Kat_Waly_Podstawy = radians(50),
Kat_Konstrukcyjny = (pi / 3 - Kat_Waly_Podstawy) / 2, Kat_Snapy_Platformy = radians(8),
Koord_Wal[2][6] = {{-98, -98, 10, 89, 89, 10},{ -46, 46, 108, 62, -62, -105}},
Koord_Snap[3][6] = {{-67, -67, -33, 98,  98, -33,}, {-76,  76,  95, 18, -18, -95,}, {  0,   0,   0,  0,   0,   0}};
//H[]-wektor translacji z podstawy do platformy.
static float Macierz_Rot[3][3], rxp[3][6], H[3] = {0, 0, H_Platformy};
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
  //inicjalizacja pinów RGB i przycisku.
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  Serial.begin(9600); //komunikacja 9600Hz
  kalibracja(Catch_X_DBound, Catch_Y_DBound, Catch_X_UBound, Catch_Y_UBound);
}
void kalibracja(int &xDBoundary, int &yDBoundary, int &xUBoundary, int &yUBoundary) {
  float xMIN = 500, yMIN = 500, yMAX = 0, xMAX = 0;
  unsigned long tim = millis();
  while (millis()-tim < 5000) {
    digitalWrite(blue,HIGH);
    digitalWrite(green,LOW);
    digitalWrite(red,LOW);
    Koordynaty(X, Y);
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
void Sortuj(int a[], int n) {
  int i, j, min, temp;
  for (i = 0; i < n - 1; i++) {
    min = i;
    for (j = i + 1; j < n; j++)
    if (a[j] < a[min])
    min = j;
    temp = a[i];
    a[i] = a[min];
    a[min] = temp;
  }}
void Koordynaty(int &a, int &b) {
  //zgodnie z algorytmem działania, ustawiam odpowiednie piny na stan wysoki - patrz do dziennika pokładowego (zielona ramka)
  //Pomiar X:
  digitalWrite(PIN_TR, LOW);
  digitalWrite(PIN_BR, LOW);
  digitalWrite(PIN_TL, HIGH);
  digitalWrite(PIN_BL, HIGH);
  //wygładzamX
  delay(2);
  while (Xi <= Ilosc_Probek) {
    delay(1);
    xSu[Xi] = analogRead(PIN_S);
    Xi++;
  }
  //filtruj mediana
  int m = sizeof(xSu)/ sizeof(xSu[0]);
  Sortuj(xSu, m);
  a = xSu[2];
  for (int i = 0; i < m; i++){
    Serial.print("**");
    Serial.print(xSu[i]);}
  xSum = 0;
  Xi = 0;
  //POMIAR Y
  digitalWrite(PIN_TR, HIGH);
  digitalWrite(PIN_TL, HIGH);
  digitalWrite(PIN_BL, LOW);
  digitalWrite(PIN_BR, LOW);
  //wygładzamY
  delay(2);
  while (Yi <= Ilosc_Probek) {
    delay(1);
    ySu[Yi] = analogRead(PIN_S);
    Yi++;
  }
  //filtruj mediana
  int n = sizeof(ySu)/ sizeof(ySu[0]);
  Sortuj(ySu, n);
  b = ySu[2];
  ySum = 0;
  Yi = 0;
}
//funkcja wyliczania żądanego kąta na serwomechanizmie
float getAlpha(int *i) {
  static int n;
  static float Przechwyc_Alfa = 0;
  static float q[3], dl[3], dl2;
  double min = Serwo_Minimum;
  double max = Serwo_Maksimum;
  n = 0;
  Przechwyc_Alfa = Kat_Orczyka[*i];
  while (n < 20) {
    //obliczenia współrzędnych połączenia pomiędzy orczykami a popychaczami.
    q[0] = Orczyk * cos(Przechwyc_Alfa) * cos(beta[*i]) + Koord_Wal[0][*i]; //Koord_Wal[][]=x y Koordynaty punktow rotacji
    q[1] = Orczyk * cos(Przechwyc_Alfa) * sin(beta[*i]) + Koord_Wal[1][*i];
    q[2] = Orczyk * sin(Przechwyc_Alfa);
    //odległość pomiedzy dwoma punktami w przetrzeni:
    dl[0] = rxp[0][*i] - q[0];
    dl[1] = rxp[1][*i] - q[1];
    dl[2] = rxp[2][*i] - q[2];
    dl2 = sqrt(dl[0] * dl[0] + dl[1] * dl[1] + dl[2] * dl[2]);
    //jeżeli dystans d12 jest równy długości popychacza obliczenia są poprawne i można liczyć dalej.
    if (abs(Popychacz- dl2) < 0.01) {
      return Przechwyc_Alfa;
    }
    //jeżeli nie, dzielę pozostałą część przez dwa i liczę dalej.
    if (dl2 < Popychacz) {
      max = Przechwyc_Alfa;
    } else {
      min = Przechwyc_Alfa;
    }
    n += 1;
    if (max == Serwo_Minimum || min == Serwo_Maksimum) {
      return Przechwyc_Alfa;
    }
    Przechwyc_Alfa = min + (max - min) / 2;
  }
  return Przechwyc_Alfa;
}
unsigned char Ustaw_Pozycje(float Poz[]) {
  unsigned char errorcount;
  errorcount = 0;
  float Psi = Poz[5];
  float Theta = Poz[4];
  float Phi = Poz[3];
  Macierz_Rot[0][0] = cos(Psi) * cos(Theta);                           //OK pierwsza kolumna pierwszy element
  Macierz_Rot[1][0] = -sin(Psi) * cos(Phi) + cos(Psi) * sin(Theta) * sin(Phi); //OK druga kolumna pierwszy element
  Macierz_Rot[2][0] = sin(Psi) * sin(Phi) + cos(Psi) * cos(Phi) * sin(Theta); //OK trzecia kolumna pierwszy element
  Macierz_Rot[0][1] = sin(Psi) * cos(Theta);                           //OK pierwsza kolumna drugi element
  Macierz_Rot[1][1] = cos(Psi) * cos(Phi) + sin(Psi) * sin(Theta) * sin(Phi); //OK druga kolumna drugi element
  Macierz_Rot[2][1] = -cos(Psi) * sin(Phi) + sin(Psi) * sin(Theta) * cos(Phi); //OK druga kolumna, trzeci element
  Macierz_Rot[0][2] = -sin(Theta);                                     //OK pierwsza kolumna trzeci element
  Macierz_Rot[1][2] =  cos(Theta) * sin(Phi);                          //OK druga kolumna trzeci element
  Macierz_Rot[2][2] = cos(Theta) * cos(Phi);                           //OK trzecia kolumna trzeci element
  
  for (int i = 0; i < 6; i++)
  {
    
    rxp[0][i] = Poz[0] + H[0] + Macierz_Rot[0][0] * (Koord_Snap[0][i]) + Macierz_Rot[0][1] * (Koord_Snap[1][i]) + Macierz_Rot[0][2] * (Koord_Snap[2][i]);
    rxp[1][i] = Poz[1] + H[1] + Macierz_Rot[1][0] * (Koord_Snap[0][i]) + Macierz_Rot[1][1] * (Koord_Snap[1][i]) + Macierz_Rot[1][2] * (Koord_Snap[2][i]);
    rxp[2][i] = Poz[2] + H[2] + Macierz_Rot[2][0] * (Koord_Snap[0][i]) + Macierz_Rot[2][1] * (Koord_Snap[1][i]) + Macierz_Rot[2][2] * (Koord_Snap[2][i]);
    
    Kat_Orczyka[i] = getAlpha(&i);
    if (i == INV1 || i == INV2 || i == INV3) { //Bierz pod uwage orientacje serwa.
      Puls_Na_Pozycji[i] = constrain(Neutralna[i] - (Kat_Orczyka[i]) * Mnoznik, MIN, MAX);
    }
    else {
      Puls_Na_Pozycji[i] = constrain(Neutralna[i] + (Kat_Orczyka[i]) * Mnoznik, MIN, MAX);
    }
  }
  for (int i = 0; i < 6; i++)
  {
    if (Kat_Orczyka[i] == Serwo_Minimum || Kat_Orczyka[i] == Serwo_Maksimum || Puls_Na_Pozycji[i] == MIN || Puls_Na_Pozycji[i] == MAX) {
      errorcount++;
    }
    servo[i].writeMicroseconds(Puls_Na_Pozycji[i]); //przepisanie.
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
  if(abs(a)>0.03 || abs(b)>0.03){
    digitalWrite(red,HIGH);
    digitalWrite(green,LOW);
    digitalWrite(blue,LOW);} else 
  {
    digitalWrite(green,HIGH);
    digitalWrite(red,LOW);
    digitalWrite(blue,LOW);}
  
}
void licz_przyspieszenie(int a, int b){
  Xa = 9.81*sin(a*deg2rad);
  Ya = 9.81*sin(b*deg2rad);
}
void kalibruj_tymczasowy(int &a, int &b){
  a += -Catch_Initial_X;
  b += -Catch_Initial_Y;
}
unsigned long ostatni_odczyt, ostatni_odczyt_calka;
void Zbierz_Koordynaty_licz_PD(void){
  
  //zdobądź Koordynata
  kalibruj_tymczasowy(X,Y);
  eX = X; //przechwycenie do zmiennej symbolicznej 
  eY = Y;//przechwycenie do zmiennej symbolicznej 
  /*nastepuje wykonanie czlonow*/
  proporcjonalny(Catch_X, Catch_Y);
  pochodna(Catch_dX, Catch_dY);
  X1 = X; Y1 = Y; //zbieram wartosc dla pochodnej.
  eX1 = X1; eY1 = Y1;//przechwycenie do zmiennej symbolicznej
}
void Wprowadz_sterowanie(void){
  Pozycja[0] = 0;
  Pozycja[1] = 0;
  Pozycja[2] = 0;
  Pozycja[3] = (Catch_Y + Catch_dY + Catch_iY);
  Pozycja[4] = (Catch_X + Catch_dX + Catch_iX);
  Pozycja[5] = radians(0);
}
void loop()
{
  stanPrzycisku = digitalRead(buttonPin);
  if (stanPrzycisku == HIGH) { Zadaj_wspolrzedne();}
    Zbierz_Koordynaty_licz_PD();
  Koordynaty(X, Y);
    calka(Catch_iX, Catch_iY);
  wskaznik(Catch_dY,Catch_dX);
  Wprowadz_sterowanie();
  Ustaw_Pozycje(Pozycja); //zmien pozycje
}
void Zadaj_wspolrzedne(){
  /*inicjalizacja punktów stabilnosci, NIE USTAWIAC GO BLIZEJ NIZ W POLOWIE ODLEGLOSCI OD SRODKA DO KRAWEDZI
  - dalej algorytm po prostu nie jest na tyle ogarniety.*/
  digitalWrite(blue,HIGH);
  digitalWrite(green,LOW);
  digitalWrite(red,LOW);
  unsigned long tim = millis();
  while (millis()-tim < 60) {
    Koordynaty(Catch_Initial_X, Catch_Initial_Y);
  }}
