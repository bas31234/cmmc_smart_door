
#define door_idle 0
#define door_active 1

const int led_pin = 13;
const int r_en_pin = 7;
const int l_en_pin = 8;
const int r_pwm_pin = 9;
const int l_pwm_pin = 10;
const int encoder_pin = 3;
const int limit_sw_pin = 2;
const int input_pin = 5;

uint8_t door_mode = door_idle;
uint8_t door_seq;
float position = 0;
float position_ref = 0;

float speed = 0;
float speed_ref = 0;
float speed_ref_2 = 0;

float kp_position = 5;
float kp_speed = 10, ki_speed = 1;

float error_position = 0;
float error_speed = 0, error_sum_speed = 0;

float motor_power = 0;

uint8_t val_limit = 0;
uint8_t val_input = 0;

uint32_t t0, t1, t2;
uint32_t prev_time_encoder;

void setup()
{

  delay(3000);
  TCCR1B = TCCR1B & (0b11111000 | 0x01); // pwm freq 16kHz

  pinMode(led_pin, OUTPUT);
  pinMode(r_en_pin, OUTPUT);
  pinMode(l_en_pin, OUTPUT);
  pinMode(r_pwm_pin, OUTPUT);
  pinMode(l_pwm_pin, OUTPUT);

  pinMode(encoder_pin, INPUT_PULLUP);
  pinMode(limit_sw_pin, INPUT_PULLUP);
  pinMode(input_pin, INPUT_PULLUP);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(encoder_pin), encoder_handle, CHANGE);

  motor_drive(40);
  while (digitalRead(limit_sw_pin))
    delay(1);
  motor_drive(0);
  position = 0;
}

void loop()
{
  if (millis() < t0)
  {
    t0 = 0;
    t1 = 0;
    t2 = 0;
    prev_time_encoder = 0;
  }
  t0 = millis();

  // position control loop 50 Hz
  if (t0 - t1 >= 20)
  {
    t1 = t0;
    static int toggle;
    toggle = !toggle;
    digitalWrite(led_pin, toggle);

    Serial.print(speed_ref);

    Serial.print("\n");

    val_input = (val_input << 1) | (0x01 & digitalRead(input_pin));

    if (!digitalRead(limit_sw_pin))
    {
      position = 0;
    }

    if (val_input == 0xF0)
    {
      speed_ref = -80;
      door_seq = 1;
    }

    switch (door_seq)
    {
    case 0:
      if (val_input == 0xF0)
      {
        door_seq++;
        speed_ref = -80;
      }
      break;
    case 1:
      if (position < -50)
      {
        speed_ref = 0;
        if (abs(speed_ref - speed_ref_2) < 2)
        {
          speed_ref = 80;
          door_seq++;
        }
      }

      break;
    case 2:
      if (position > -15)
      {
        speed_ref = 30;
        if (abs(speed_ref - speed_ref_2) < 2)
        {
          door_seq++;
        }
      }

      break;
    case 3:

      while (digitalRead(limit_sw_pin))
        delay(1);
      motor_drive(0);
      speed_ref = 0;
      speed_ref_2 = 0;
      door_seq = 0;
      break;
    }

    if (speed_ref_2 > speed_ref)
    {
      speed_ref_2 -= 1;
    }
    if (speed_ref_2 < speed_ref)
    {
      speed_ref_2 += 1;
    }

    motor_drive(speed_ref_2);
  }
}

void motor_drive(int val)
{
  val = constrain(val, -100, 100);
  if (abs(val) <= 2)
  {
    digitalWrite(r_en_pin, LOW);
    digitalWrite(l_en_pin, LOW);
  }
  else
  {
    digitalWrite(r_en_pin, HIGH);
    digitalWrite(l_en_pin, HIGH);

    if (val > 0)
    {
      analogWrite(l_pwm_pin, 0);
      analogWrite(r_pwm_pin, 2.5 * val);
    }
    else
    {
      analogWrite(r_pwm_pin, 0);
      analogWrite(l_pwm_pin, -2.5 * val);
    }
  }
}

void encoder_handle(void)
{
  if (!digitalRead(limit_sw_pin))
  {
    position = 0;
    speed = 0;
  }
  else
  {
    float dt = millis() - prev_time_encoder;
    if (dt >= 2) //debounce
    {
      prev_time_encoder = millis();

      //position measuring
      if (speed_ref_2 >= 2)
        position++;

      if (speed_ref_2 <= -2)
        position--;
    }
  }
}

void limit_sw_handle(void)
{
}

float lpf(float alpha, float val_in, float val_out)
{
  return val_out + alpha * (val_in - val_out);
}