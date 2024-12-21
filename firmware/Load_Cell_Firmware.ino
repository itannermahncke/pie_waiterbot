const int fl_pin = A0;
const int fr_pin = A1;
const int bl_pin = A2;
const int br_pin = A3;

int fl_value = 0;
int fr_value = 0;
int bl_value = 0;
int br_value = 0;

const int fl_low_calib = 60;
const int fr_low_calib = 10;
const int bl_low_calib = 10;
const int br_low_calib = 15;

const int fl_high_calib = 100;
const int fr_high_calib = 10;
const int bl_high_calib = 10;
const int br_high_calib = 35;

int old_weight_state = 0;
int new_weight_state = 0;

int state_list[5] = {0,0,0,0,0};

int old_change_state = 2;
int new_change_state = 2;

bool stable = false;

void setup() {
  Serial.begin(115200);

}

void loop() {
  fl_value = analogRead(fl_pin);
  fr_value = analogRead(fr_pin);
  bl_value = analogRead(bl_pin);
  br_value = analogRead(br_pin);

/*
  Serial.println(fl_value);
  Serial.println(fr_value);
  Serial.println(bl_value);
  Serial.println(br_value);
  */
  
  if (fl_value <= fl_low_calib && fr_value <= fr_low_calib && bl_value <= bl_low_calib && br_value <= br_low_calib)
  {
    new_weight_state = 1;
  }
  else if (fl_value >= fl_high_calib || fr_value >= fr_high_calib || bl_value >= bl_high_calib || br_value >= br_high_calib)
  //else if (fl_value >= fl_high_calib && br_value >= br_high_calib)
  {
    new_weight_state = 3;
  }
  else
  {
    new_weight_state = 2;
  }

  state_list[0] = new_weight_state;

  if (state_list[0] == state_list[1] && state_list[0] == state_list[2] && state_list[0] == state_list[3] && state_list[0] == state_list[4])
  {
    stable = true;
  }
  else
  {
    stable = false;
  }

  if (old_weight_state == 1 && new_weight_state == 2 && stable == true)
  {
    new_change_state = 1;
  }
  else if (old_weight_state == 2 && new_weight_state == 3 && stable == true)
  {
    new_change_state = 2;
  }
  else if (old_weight_state == 2 && new_weight_state == 1 && stable == true)
  {
    new_change_state = 3;
  }
  else if (old_weight_state == 3 && new_weight_state == 2 && stable == true)
  {
    new_change_state = 4;
  }
  
  if (old_change_state == 2 && new_change_state == 4)
  {
    Serial.print("02");
    Serial.print("sg");
    Serial.println("1");
  }
  //else if (old_change_state == 1 && new_change_state == 2)
  //{
    //Serial.print("02");
    //Serial.print("sg");
    //Serial.println("1");
  //}

  else
  {
    
    Serial.print("02");
    Serial.print("sg");
    Serial.println("0");
    
  }

  state_list[4] = state_list[3];
  state_list[3] = state_list[2];
  state_list[2] = state_list[1];
  state_list[1] = state_list[0];

  if (stable == true)
  {
  old_weight_state = new_weight_state;
  }
  old_change_state = new_change_state;

  /*
  Serial.print(fl_value);
  Serial.print(",");
  Serial.print(fr_value);
  Serial.print(",");
  Serial.print(bl_value);
  Serial.print(",");
  Serial.println(br_value);
  Serial.print(state_list[0]);
  Serial.print(state_list[1]);
  Serial.print(state_list[2]);
  Serial.print(state_list[3]);
  Serial.println(state_list[4]);
  Serial.println(new_change_state);*/
  

  delay(100);

}
