int pir_pin=13;
int buzzer_pin=2;

void setup()
{
pinMode(pir_pin,INPUT);
pinMode(buzzer_pin,OUTPUT);
}

void loop()
{
  int motion=0;
  motion = digitalRead(pir_pin); 
  if (motion==HIGH){
   digitalWrite(buzzer_pin,HIGH); 
  }
  
  else {
   digitalWrite(buzzer_pin,LOW);   
  }

  delay(10);
  
}