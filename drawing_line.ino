double inputDegree1, inputDegree2;
double current_time = 0;
double th1_new, th2_new, th1_old, th2_old;
double th1, th2;
double end_time, start_time;
double x, y, x_old, y_old;
double th1_0 = 82, th2_0 = 122;
double slope, C, dx, x_new, y_new;



//============== SETUP FUNCTION ==============
void setup() {
  pinMode(5, OUTPUT);           // servo1
  pinMode(6, OUTPUT);           // servo2
  Serial.begin(9600);
  noInterrupts();
  TIME_INTERRUPT_REGISTER();    // register
  interrupts();
  timer_init();                // setting current_timer

}



//============== LOOP FUNCTION ===============
void loop() {
}


//============== ISR FUNCTION ================
ISR(TIMER1_COMPA_vect)
{
  current_time += 0.01;
  drawLine(10, 12, 5, 5);
}


//=============== CIRCLE ===================

void drawLine(double x_old, double y_old, double x, double y) {
  slope =  (y-y_old)/(x-x_old);
  C = y_old - slope*x_old;
  dx = (x - x_old)/10;
  x_new = x_old;
  for(int i=0; i<10; i+=0.1){
    x_new = x_new + dx*i;
    y_new = slope*x_new + C;
    invKinematic(x_new, y_new, &th1, &th2);
    servoOut(th1, th2);
    Serial.print(th1);
    Serial.print(", ");
    Serial.println(th2);
    delay(1);
  }
}


//================ ROBOT ====================

void robot(double current_time) {

  if (current_time <= 3) {
    x = 5.0;
    y = 15.0;
    start_time = 0;
    end_time = start_time + 3;
    go2xy(x, y, start_time, current_time, end_time, th1_0, th2_0, &inputDegree1, &inputDegree2, &th1_old, &th2_old);
  }
  else if (current_time < 34) {
    inputDegree1 = one_cos(32, current_time, th1_old, th1_0, 2);
    inputDegree2 = one_cos(32, current_time, th2_old, th2_0, 2);
  }

  if (current_time < 34) {
    Serial.print(inputDegree1);
    Serial.print(" ,");
    Serial.println(inputDegree2);
  }
  servoOut(inputDegree1, inputDegree2);
}


//============== TIME INTERRUPT ==============
void TIME_INTERRUPT_REGISTER()
{
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  TCNT1 = 0;
  OCR1A = 16000 - 1;
  TIMSK1 |= (1 << OCIE1A);
}

//================ TIMER INIT =================
void timer_init() {    // pin - 5
  DDRB |= 0b01100000;
  TCCR3A = 0b10100010;
  TCCR3B = 0b00011010;
  ICR3 = 39999;
  //OCR3A = 2999;  // 90 deg - 2999  - pin 5
  OCR3A = (int)(999 + (4999 - 999) / 180 * th1_0);
  DDRB |= 0b01100000;
  TCCR4A = 0b10100010;
  TCCR4B = 0b00011010;
  ICR4 = 39999;
  //OCR4A = 999;   // 0 deg - pin 6
  OCR4A = (int)(999 + (4999 - 999) / 180 * th2_0);
}

//================ SERVO OUT =================
void servoOut(double deg1, double deg2) {  // deg 1 (pin - 5, link - 1),  deg 2 (pin - 6, link - 2)
  if (deg1 > 180) deg1 = 180;
  else if (deg1 < 0 ) deg1 = 0;
  if (deg2 > 180) deg2 = 180;
  else if (deg2 < 0 ) deg2 = 0;
  int duty1 = (int)(999 + (4999 - 999) / 180 * deg1);   // 999 -> 1ms, 4999 -> 2ms
  int duty2 = (int)(999 + (4999 - 999) / 180 * deg2);
  OCR3A = duty1;
  OCR4A = duty2;
}


//======================== ONE - COS =========================
double one_cos(double t_0, double t, double INIT, double FINAL, double T) {
  // t_0 - starting time, t - current_time, init - init position,
  // final - final position, T - half the period
  double f = 1 / (2 * T);
  double ref = ((FINAL - INIT) / 2 * (1 - cos(2 * 3.14 * (t - t_0) * f))) + INIT;
  return ref;
}




//==================== INVERSE KINEMATIC ======================
void invKinematic(double x, double y, double* th1, double* th2) {
  x = x / 100;
  y = y / 100;
  double L1 = 0.15, L2 = 0.15;
  *th2 = PI - acos( (L1 * L1 + L2 * L2 - x * x - y * y) / (2 * L1 * L2) );
  *th1 = atan2(y, x) - atan2( L2 * sin(*th2), L1 + L2 * cos(*th2) );
  *th1 = (*th1 * 180 / PI) + 90;
  *th2 = *th2 * 180 / PI;
}

// ========================== GOTOXY ============================

void go2xy(double x, double y, double start_time, double current_time, double end_time, double th1_0, double th2_0, double* inputDegree1, double* inputDegree2, double* th1_old, double* th2_old) {
  invKinematic(x, y, &th1_new, &th2_new);
  *inputDegree1 = one_cos(start_time, current_time, th1_0, th1_new, end_time - start_time);
  *inputDegree2 = one_cos(start_time, current_time, th2_0, th2_new, end_time - start_time);
  if (current_time > end_time - 0.01) {
    *th1_old = th1_new;
    *th2_old = th2_new;
  }
}
