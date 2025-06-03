/* 
  Αόρατη Άρπα
  Βασισμένο στο https://www.hackster.io/michalin70/build-a-laser-harp-with-a-lidar-and-arduino-a35cdc

  Ένα Arduino και ένα LiDAR που ανιχνεύουν τα χέρια του οργανοπαίκτη και δημιουργούν νότες!

  Συνδέουμε την σειρική έξοδο (RX/TX) ενός RPLIDAR σε μια σειριακή είσοδο ενός Arduino Mega.
  Συνδέουμε την είσοδο ελέγχου κινητήρα (CTRL_MOTO) του LiDAR σε μια έξοδο PMW (ή κατευθείαν στα 5v).
  Το LiDAR τροφοδοτείται από τα 5v του arduino και φυσικά συνδέουμε την γειώσεις τους (GND).
  Προσοχή: 
  Το RPLIDAR απαιτεί τουλάχιστον 800mA στην εκκίνηση, οπότε ίσως το αρντουίνο χρειαστεί εξωτερική τροφοδοσία.
  Τα γιαλυστερά αντικείμενα δεν ανιχνεύονται.
  Οι νότες παράγονται με τη δημιουργία της ανάλογης ημιτονοειδούς κυματομορφής,
  που εξάγεται σε μια ακίδα εξόδου με χρήση Διακοπών (Interrupts).
  Επίσεις η ένταση κάθε νότας φθίνει στον χρόνο.
  
  Air Harp, with an Arduino using a Lidar to scan the hands.
  Connect the RPLIDAR's serial port (RX/TX) to your Arduino board serial.
  Connect motor control pin (CTRL_MOTO) to an PWM pin, (or straight to 5v).
  power supply 800mA at startup.  also arduino Gnd and Lidar Gnd have to be connected.
  REMEMBER It hates glossy surfaces.
  Includes RPLidar library: https://github.com/robopeak/rplidar_arduino, as file RPLidar.h
*/
#include "RPLidar.h"

RPLidar lidar;

const byte num_samples = 156;
int instrument_samples[num_samples];

constexpr byte lidarMotorCtrlPin = 3;       // PWM pin controling motor speed
constexpr byte buzzer_pin = 11;

void setup() {
	Serial.begin( 500000 );       //2000000 (at least 460800 baud rate to avoid faulty measurements)
	Serial.println(F("\nSerial to monitor OK"));
	
	lidar.begin(Serial1);                     // bind the RPLIDAR driver to another hardware serial
	Serial1.begin( 115200 );                  // Lidar A1M8 baud rate is 115200 
	pinMode(lidarMotorCtrlPin, OUTPUT);
	analogWrite(lidarMotorCtrlPin, 255);      // Full rotation speed
	Serial.println(F("Serial to lidar OK"));
	delay(1000);
	
	uint8_t duty_cycle = 50;
	uint16_t nDuty = (duty_cycle * num_samples) / 100;
	
	for (int i = 0; i < num_samples; i++) {
	instrument_samples[i] = 26  * sin(20 * PI * (float)i / num_samples);
	}

	// Setup the Timer 1 PWM Waveform Generation, by direct Registers addressing.
	
	// Port B Data Direction Register
	DDRB |= (1 << PB5); // Aka pinMode(11, OUTPUT)
	// Port B5: OC1A/PCINT5 (Output Compare and PWM Output A for Timer/Counter1 or Pin Change Interrupt 5)
	// Timer 1 Control Registers A & B
	TCCR1A = (1 << COM1A1) // Compare Match Output A Mode bits (COMnA): Toggle OC1A on compare match
	+ (1 << WGM11) + (0 << WGM10); // Waveform Generation Mode bits (WGM): Fast-PWM 9-Bit
	TCCR1B = (0 << CS12) + (0 << CS11) + (1 << CS10); // Clock Select bits (CS): No prescaling 
	// Timer 1 Interrupt Mask Register
	TIMSK1 = (1 << TOIE1); // Timer 1 Overflow Interrupt Enable (TOIE1)
	// OC1A (Output Compare Register) Frequency = (Arduino Frequency clock: 16MHz) / 256 = 62500 Hz
	// An overflow occurs every 16 microseconds. Then an interupt.

	Serial.println(F("ENARKSH BROXOY.."));
}
constexpr uint16_t min_distance = 15;
constexpr uint16_t max_distance = 40;
constexpr float harp_sector = 180;
constexpr float min_angle = 180 - harp_sector / 2;
constexpr float max_angle = 180 + harp_sector / 2;
constexpr byte num_strings = 8;                             // 8 χορδές
constexpr float string_sector = harp_sector / num_strings;  // γωνία νότας = 180° / 8 = 22,5°

int stringNo;
float freq_kHz;
byte vol;
auto prev_moment{ millis() };

void loop(){ 
  if ( !IS_OK( lidar.waitPoint() ) ) {  // async wait for a sample point to arrive, default timeout is 0,5 sec
	  reset_lidar();
    return;
  }

  constexpr int vol_decay = 50;         // microsec of delay of the decay of volume
  if (vol > 0) {                        // Check if a string is playing then reduce volume.
    vol--;
    delay(vol_decay);
    // Serial.println((String)F("vo: ") + vol );
    return;
  }

  const auto point = lidar.getCurrentPoint();       // Measurements are NOT ordered.
  const uint16_t distance_cm = point.distance / 10; // distance value from mm to cm unit
  const float angle_deg = point.angle;              // anglue value in positive degrees (CW from heading)
  const uint8_t quality = point.quality;            // quality of the this measurement
  const bool startBit = point.startBit;             // does this point belong to a new scan?

  // Initial Filtering: Reject possibly bad meassurments and distance beyond a max of 100 cm.
  if (quality != 15
  || distance_cm < min_distance  
  || distance_cm > max_distance)
    return;

  // Initial Filtering: Reject angles outside a predefined sector of 90Â°, centered around 180Â° of LiDAR.
  if (  angle_deg < min_angle
     || angle_deg > max_angle ) {
    return;
  }
  Serial.print( (String)F("\n startBit: ") + startBit + F(", Q: ") + quality + F(", D: ") 
    + distance_cm + F(" cm, ") + (int)angle_deg + F("Â° ") );

  // According to the angle, find the string played out of a number of strings.
  stringNo = angle_to_stringNo(angle_deg);
  // Reject margins between string notes
  if (stringNo == 0)
    return;
  Serial.print( (String)F("-> XORDH # ") + stringNo + F(" -> ") );
  print_music_note(stringNo);

  // Freq in kHz = 0.44 kHz x 2^( n/ (24/2) ), where n is the number of semitones above or below note A4.
  // freq_kHz = 0.44 * pow(2.0, ((float)string_no + 36 - 49) / 12);
  freq_kHz = 0.44 * pow( 2.0, (float)(stringNo - 5) / (num_strings / 1) );
  vol = 11;                 // The volume decay down of max volume 10.
  // Serial.print((String)F(" -> ") + (int)(freq_kHz * 1000) + F(" Hz") );
}

// To drive some regular event we use AVR Interrupt Service Routine Timer1 timer overflow interrupt,
// that fires when an OVERFLOW event occurs.
ISR(TIMER1_OVF_vect) {
  // Phase accumulator, stores the phase of the sine wave you are at, and use it as the index to your table.
  static uint32_t overflows_counter;
  // At regular intervals (timer interrupt) add a value to it, then output the new phase to the DAC.
  overflows_counter++;
  // Generate PWM signal representing sinusoidal wave to imitate a harp waveform.
  OCR1A = 256 + vol * instrument_samples[(uint16_t)(overflows_counter * freq_kHz) % num_samples];
}

byte angle_to_stringNo(const float angle_deg) {
  const float relative_angle = angle_deg - min_angle;
  const byte stringNo =  1 + relative_angle / string_sector;
  // No string note is above string note upper margin
  constexpr float string_margin = 1.25; //  
  // if (relative_angle > stringNo * string_sector - string_margin)
  //   return 0;
  return stringNo;  
}

void print_music_note(const byte string_no) {  
  static String note;
  switch (string_no) {
    case 1:
      note = "NTO";
      break;
    case 2:
      note = "RE";
      break;
    case 3:
      note = "MI";
      break;
    case 4:
      note = "FA";
      break;
    case 5:
      note = "SOL";
      break;
    case 6:
      note = "LA";
      break;
    case 7:
      note = "SI";
      break;
    case 8:
      note = "NTO";
  }
  Serial.print(note);
}

void reset_lidar() {
  analogWrite(lidarMotorCtrlPin, 0); //stop the motor
  Serial.println(F("Motor stopped, trying to detect the LIDAR.."));
  rplidar_response_device_info_t info;
  while (true) {
    delay(2000);
    if ( IS_OK(lidar.getDeviceInfo(info)) ) {  // default timeout is 1 sec
      Serial.println(F("Lidar detected. Start scan"));
      lidar.startScan();   
      analogWrite(lidarMotorCtrlPin, 255); // rotate motor at max speed
      Serial.println(F("Motor started!"));
      delay(2000);
      return;
    }
    else
      Serial.println("LIDAR not detected. Trying again...");
  }
}
