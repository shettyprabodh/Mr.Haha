
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and BMA150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "BMA150.h"
#include "HMC5883L.h"
#include "ITG3200.h"

#define M_PI 3.14159265359

enum Operation {
  REST, FORWARD, REVERSE, LEFT, RIGHT
};

Operation transmit_value = REST;

class KeyStroke {  
public:
  enum SplKey {
    NONE, ALT, CTRL, SHIFT, AR_LEFT, AR_RIGHT, AR_UP, 
    AR_DOWN, ENTER, BKSP  };

private:
  // If false, it is a special key
  bool char_key;
  char char_key_code;
  SplKey spl_key_code;

public:
  KeyStroke()
: 
    char_key(false),
    char_key_code('\0'),
    spl_key_code(NONE)
    {
    }

  void set(char c) {
    char_key = true;
    char_key_code = c;
    spl_key_code = NONE;
  }

  void set(SplKey k) {
    char_key = false;
    char_key_code = '\0';
    spl_key_code = k;
  }

  void unset() {
    char_key = false;
    char_key_code = '\0';
    spl_key_code = NONE;
  }

  void set_from_key_code(int code) {
    Serial.print("Key code: ");
    Serial.println(code);
    if (code < 26)
      set((char)code + 'a');
    else
     set('x');
  }

  void serial_print() {
    if (char_key) {
      Serial.print("+");
      Serial.println(char_key_code);
    }
    else {
      Serial.print("-");
      Serial.println(spl_key_code);
    }
  }

  // true if this contains a valid keystroke
  bool valid() {
    if (char_key == false && spl_key_code == NONE)
      return false;
    return true;
  }
};

class InputProtocolManager {
  enum InputMode {
    SELECT, CONTROL, KEYBOARD, MOUSE, ROBOT  };
  InputMode mode;

  int input_buf[3];
  int input_buf_length;
  KeyStroke next_key;
  
  double pitch;
  double roll;
  double yaw;
  double pitch_acc;
  double roll_acc;
  double yaw_acc;
  
  uint32_t timer;

  int get_accel_code(int16_t x, int16_t y, int16_t z) {
    if (abs(x) > abs(y) && abs(x) > abs(z)) 
      return (x > 0)?0:1;
    if (abs(y) > abs(x) && abs(y) > abs(z))
      return (y > 0)?2:3;
    if (abs(z) > abs(y) && abs(z) > abs(y))
      return (z > 0)?4:5;
  }

  int16_t get_input_buf_int() {
    switch (input_buf_length) {
    case 0:
      return -1;
    case 1:
      return input_buf[0];
    case 2:
      return input_buf[0]*6 + input_buf[1];
    case 3:
      return input_buf[0]*36 + input_buf[1]*6 + input_buf[0];
    }
    return -1;
  }

  void register_input_code(int code) {
    int input_buf_int;
    switch (mode) {
    case SELECT:
      switch (code) {
      case 0: 
        mode = SELECT; 
        break;
      case 1: 
        mode = CONTROL; 
        break;
      case 2: 
        mode = KEYBOARD; 
        break;
      case 3: 
        mode = MOUSE; 
        break;
      case 4: 
        mode = ROBOT; 
        break;
      }
      break;
    case CONTROL:
      switch (code) {
      case 0: 
        next_key.set(KeyStroke::BKSP); 
        break;
      case 1: 
        next_key.set(KeyStroke::AR_LEFT); 
        break;
      case 2: 
        next_key.set(KeyStroke::AR_RIGHT); 
        break;
      case 3: 
        next_key.set(KeyStroke::AR_UP); 
        break;
      case 4: 
        next_key.set(KeyStroke::AR_DOWN); 
        break;
      case 5: 
        mode = SELECT; 
        break;
      } 
      break;
    case KEYBOARD:
      input_buf_int = get_input_buf_int();
      if (input_buf_length < 2)
        input_buf[input_buf_length ++] = code;
      else if (input_buf_length == 2) {
        if (input_buf_int > 21)
          input_buf[input_buf_length ++] = code;
        else {
          next_key.set_from_key_code(input_buf_int);
          input_buf_length = 0;
          if (input_buf_int == 0)
            mode = SELECT;
        }
      }
      else {
        input_buf_length = 0;
        if (input_buf_int == 0)
          mode = SELECT;
        else
          next_key.set_from_key_code(input_buf_int);
      }
    break;
    case ROBOT:
      if (code < 5) {
        Serial.print("Robot: ");
        Serial.println(code);
        Serial1.println(code);
      }
      else 
        mode = SELECT;
    break;
    default:
      //Serial.print(mode);
      //Serial.println(" mode not yet implemented. Switching back to SELECT");
      //mode = SELECT;
    break;
    }
  }

public:
  InputProtocolManager()
  : pitch(0.0),
    roll(0.0),
    yaw(0.0),
    pitch_acc(0.0),
    roll_acc(0.0),
    yaw_acc(0.0),
    timer(micros())       //Initialize timer
  {}
  
  void register_input (int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    if (mode != MOUSE) {
      register_input_code(get_accel_code(ax, ay, az));
      Serial.print("Code: ");
      Serial.println(get_accel_code(ax, ay, az));
    }
    else {
      Serial.print('$');
      Serial.print(pitch); Serial.print("\t");
      Serial.print(roll); Serial.print("\t");
      Serial.println(yaw);
    }
    // Estimate orientation from gyroscope
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
    pitch += ((double)gx / 14.375) * dt;            //Raw gyro data
    roll += ((double)gy / 14.375) * dt;
    yaw += ((double)gz / 14.375) * dt;
    
    int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);
    if (forceMagnitudeApprox > 200 && forceMagnitudeApprox < 500)
    {
      pitch_acc = atan2((double)az, (double)ay) * 180 / M_PI;          //Raw accel data
      roll_acc = atan2((double)ax, (double)az) * 180 / M_PI;
      yaw_acc = atan2((double)ay, (double)ax) * 180 / M_PI;
    
      pitch = pitch * 0.98 + pitch_acc * 0.02;
      roll = roll * 0.98 + roll_acc * 0.02;
      yaw = yaw * 0.98 + yaw_acc * 0.02;
    }
  }

  void serial_print() {
    if (next_key.valid()) {
      next_key.serial_print();
      next_key.unset();
    }
    else {
      Serial.print ("Mode: ");
      Serial.println(mode);
    }
  }
  
  int16_t get_loop_delay() {
    if (mode == MOUSE)
      return 100;
    else if(mode == ROBOT)
      return 1000;
    else
      return 500;
  }
  
  bool wait_for_button() {
    if (mode == MOUSE)
      return false;
     return true;
  }
};

// class default I2C address is 0x38

BMA150 accel;
HMC5883L compass;
ITG3200 gyro(0x69);

int16_t ax, ay, az;
int16_t cx, cy, cz;
int16_t gx, gy, gz;

int x_trans = 0, n = 0;                 //For Bot control

uint32_t timer;

InputProtocolManager input_manager;

const int button_pin = 36;

void setup() {
  pinMode(button_pin, INPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);

  // Initialize devices
  //Serial.println("Initializing I2C devices...");
  accel.initialize();
  compass.initialize();
  gyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "BMA150 connection successful" : "BMA150 connection failed");
  Serial.println(compass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  Serial.println(gyro.testConnection() ? "ITG3200 connection successful" : "ITG3200 connection failed");
  
  accel.getAcceleration(&ax, &ay, &az);
  gyro.getRotation(&gx, &gy, &gz);
  
  //pitch_acc = atan2((double)az, (double)ay) * 180 / M_PI;
  //roll_acc = atan2((double)ax, (double)az) * 180 / M_PI;
  //yaw_acc = atan2((double)ay, (double)ax) * 180 / M_PI;
  
}

void loop() {
 if (digitalRead(button_pin) || !input_manager.wait_for_button()) {
    // read raw gyro measurements from device
   
    
    accel.getAcceleration(&ax, &ay, &az);
    //Serial.println(get_accel_code(ax, ay, az));
    //input_manager.register_input(ax, ay, az);
    //input_manager.serial_print();
    //compass.getHeading(&cx, &cy, &cz);
    gyro.getRotation(&gx, &gy, &gz);
    
    
    
    //Serial.print("Pitch, Roll, Yaw:\t");
    //Serial.print(pitch); Serial.print("\t");
    //Serial.print(roll); Serial.print("\t");
    //Serial.println(yaw);
    //Serial.println(x_trans);
    
    //Serial.print("accel:\t");
    //Serial.print(pitch_acc); Serial.print("\t");
    //Serial.print(roll_acc); Serial.print("\t");
    //Serial.println(yaw_acc);
    
    //Serial.println(forceMagnitudeApprox);
    // display tab-separated accel x/y/z values
    //Serial.print("accel:\t");
    //Serial.print(ax); Serial.print("\t");
    //Serial.print(ay); Serial.print("\t");
    //Serial.println(az);
      
    input_manager.register_input(ax, ay, az, gx, gy, gz);
    input_manager.serial_print();
    delay(input_manager.get_loop_delay());
 }
  // display tab-separated accel x/y/z values
  //    Serial.print("accel:\t");
  //    Serial.print(ax); Serial.print("\t");
  //    Serial.print(ay); Serial.print("\t");
  //    Serial.println(az);
}



