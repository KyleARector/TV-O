int ledPin1 = 2;
int ledPin2 = 13;
int state=0;
void setup() {
    pinMode(ledPin1, OUTPUT); 
    pinMode(ledPin2, OUTPUT); 
    Serial.begin(9600); 
}

void loop() {
  if (Serial.available() > 0)
  {
    state = Serial.read(); 
    
    switch(state)
    {
      case '3': // On
        digitalWrite(ledPin2,HIGH);
        digitalWrite(ledPin1,HIGH);
      break;
      case '2': // On
        digitalWrite(ledPin2,HIGH);
        digitalWrite(ledPin1,LOW);
      break;
      case '1': // On
        digitalWrite(ledPin2,LOW);
        digitalWrite(ledPin1,HIGH);
      break;
      case '0': // Off
        digitalWrite(ledPin1,LOW);
        digitalWrite(ledPin2,LOW);
      break;
      default:
      break; 
    }
  }
}

