class Stepper {
public:
  int startPin;
  int activeIndex;
  int state;
  int msPerStep;

  Stepper(int startPin) : startPin(startPin) {
    for (int i = 0; i < 4; ++i)
      pinMode(startPin+i, OUTPUT);

    activeIndex = 0;
    state = 0;
    msPerStep = 30;
  }

  void dw(int index, int output) {
    digitalWrite(startPin + index, output);
  }

  void step(char d) {
    int oldAI = activeIndex;

    activeIndex = (activeIndex + d) & 3;
    state += d;
    
    dw(activeIndex, HIGH);
    delay(msPerStep/2);
    dw(oldAI, LOW);
    delay(msPerStep/2);
  }

  void target(int t) {
    dw(activeIndex, HIGH);
    
    while (t > state) step(+1);
    while (t < state) step(-1);

    dw(activeIndex, LOW);
  }
};

Stepper a(38);
Stepper b(42);

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

int led = 0;
int v = 0;
void loop() {
  digitalWrite(13, led ? HIGH : LOW);
  led = !led;

  v += 100;
  a.target(v);
  b.target(v);

  delay(1000);
}
