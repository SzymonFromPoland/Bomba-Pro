#include <motors.h>

void drive(int left, int right)
{
  left = constrain(left, -100, 100);
  right = constrain(right, -100, 100);

  int pwmLeft = map(abs(left), 0, 100, 0, 255);
  int pwmRight = map(abs(right), 0, 100, 0, 255);

  if (left > 0)
  {
    digitalWrite(m1a, HIGH);
    digitalWrite(m1b, LOW);
  }
  else if (left < 0)
  {
    digitalWrite(m1a, LOW);
    digitalWrite(m1b, HIGH);
  }
  else
  {
    digitalWrite(m1a, LOW);
    digitalWrite(m1b, LOW);
  }

  if (right > 0)
  {
    digitalWrite(m2a, HIGH);
    digitalWrite(m2b, LOW);
  }
  else if (right < 0)
  {
    digitalWrite(m2a, LOW);
    digitalWrite(m2b, HIGH);
  }
  else
  {
    digitalWrite(m2a, LOW);
    digitalWrite(m2b, LOW);
  }

  analogWrite(pwm1, pwmLeft);
  analogWrite(pwm2, pwmRight);
}