const int N_STEPS = 200;
const int N_POSITIONS = 8; // nb must be an even number

int n =0;

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));
}

void loop() {
  Serial.println(getMoveDistance());
  delay(100);
}

int getMoveDistance(){
  int r = (random(N_POSITIONS/2) + 1) * (N_STEPS / N_POSITIONS);
  return random(2) ? r : -r;
}
