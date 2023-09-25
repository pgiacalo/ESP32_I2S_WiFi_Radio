/**
 * This software is written for the ESP32 microcontroller. It follows Arduino coding conventions 
 * with functions setup() and loop(). 
 * This software creates an audio player that gets streaming audio data from internet URLs 
 * using a connection to a WiFi network. 
 * Your WiFi network's ssid and password must be configured in the code, before compiling and 
 * uploading the code to your ESP32. 
 * 
 * Notes about the starup process and the LED indicators:
 * 1) At startup, the ESP32 LED blinks once per second while attempting to connect to WiFi.
 * 2) If WiFi connectivity succeeds, the blinking LED will turn off. 
 * 3) If WiFi connectivity fails, the LED will blink S.O.S. continuously to indicate a startup failure. 
 *      - Double check the values of the WiFi ssid and password in the code, if WiFi connectivity fails. 
 * 
 * Messages are printed to the Serial Monitor to show startup progress or failures.
 *
 * These youtube videos explain the code, wiring, volume control, channel control, etc.
 * https://www.youtube.com/watch?v=t4K1HBQUj-k
 * https://www.youtube.com/watch?v=vFC1nT9BRMs
 * https://www.youtube.com/watch?v=NlelI2dgCPU
 *****************************************************************************************
 * NOTES on MAX98357A wiring to setup the left and right audio channels:
 *    REFERENCE: https://learn.adafruit.com/adafruit-max98357-i2s-class-d-mono-amp/pinouts
 *
 *    LEFT CHANNEL:  If the voltage on SD is greater than 1.4V then the output is just the Left channel.
 *    RIGHT CHANNEL: If the voltage on SD is between 0.77V and 1.4V then the output is just the Right channel.
 *
 *    There is an internal 100K pulldown resistor on SD so you need to 
 *    use a pullup resistor on SD (to balance out the pulldown resistor).
 *        -- with a 5v supply, I use a 100 ohm pullup resistor on the LEFT channel and 470 ohm on the RIGHT.
 *
 *    MONO:  If the voltage on SD is between 0.16V and 0.77V then the output is (Left + Right)/2, that is the stereo average. 
 *    SHUTDOWN:  If SD is connected to ground directly (voltage is under 0.16V) then the amp is shut down
 *            to ground directly (voltage is under 0.16V) then the amp is shut down
 *
 *****************************************************************************************
 *
 * Special thanks to Wolle schreibfaul1 ESP32-audioI2S library (https: *github.com/schreibfaul1)
 *
 * Requires the following ESP32 I2S audio library from github
 * https: *github.com/schreibfaul1/ESP32-audioI2S
 * The associated wiki page for this library is found here:
 * https://github.com/schreibfaul1/ESP32-audioI2S/wiki
 ******************************************************************************************
 */

#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include <ezButton.h>

//ESP32 I2S digital output pins
#define I2S_DOUT      25  //GPIO 25 (DATA Output - the digital output. connects to DIN pin on MAX98357A I2S amplifier)
#define I2S_BCLK      26  //GPIO 26 (CLOCK Output - serial clock. connects to BCLK pin on MAX98357A I2S amplifier)
#define I2S_LRC       27  //GPIO 27 (SELECT Output - left/right control. connects to LRC pin on MAX98357A I2S amplifier)

//ESP32 analog input (WARNING: limit voltage to 3.3v maximum)
#define POT_PIN             34  //GPIO 34   //input pin used to control the audio volume

#define CHAN_UP_PIN         4   //increases the channel number
#define CHAN_DOWN_PIN       0   //decreases the channel number
#define DEBOUNCE_TIME 50

#define NUMBER_OF_CHANNELS  10  //this should match the number of URLs found in the connect() function below

#define LED_PIN       2   //GPIO 2    //used by the code to control the ESP32's blue LED 

#define VOLUME_CONTROL_STEPS  100     //100 steps -- the potentiometer (on GPIO34) controls audio volume between zero and 100%
#define WIFI_MAX_TRIES        5      //number of attempts to connect to WiFi during startup

ezButton upButton(CHAN_UP_PIN);
ezButton downButton(CHAN_DOWN_PIN);

int currentChannelNumber = 1;

//WiFi account login
const String ssid         = "Aardvark";   //wifi network name 
const String password     = "";   //wifi password

Audio audio;  //class from the ESP32-audioI2S library

void setupAudio(){
  Serial.println("Setting I2S output pins and volume level.");

  //configure the I2S output pins
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);

  //set the initial volume level
  audio.setVolumeSteps(VOLUME_CONTROL_STEPS);
  int volume = map(analogRead(POT_PIN), 0, 4095, 0, VOLUME_CONTROL_STEPS);  // map potentiometer value to a volume percentage
  audio.setVolume(volume);
  Serial.print("Volume set at ");
  Serial.print(volume);
  Serial.println("%");
}

void connectWiFi(){
  WiFi.disconnect();

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid.c_str(), password.c_str());

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < WIFI_MAX_TRIES){
      tries++;
      Serial.print("Attempting to connect to WiFi. Try #");
      Serial.println(tries);
      //blink the LED while we are trying to connect
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
  }

  if (WiFi.status() == WL_CONNECTED){
    //WiFi connection succeeded. Turn the LED OFF. 
    digitalWrite(LED_PIN, LOW);
    Serial.print("SUCCESS: connected to wifi network ");
    Serial.println(ssid);
  } else {
    //WiFi connection FAILED. Blink S.O.S. with the LED.  
    while(true){
      Serial.print("WiFi error: Failed to connect to ");
      Serial.println(ssid);
      Serial.println("------------------------------------");
      Serial.println("RE-CHECK YOUR WiFi SSID and PASSWORD");
      Serial.println("------------------------------------");
      blinkSOS(5);
    }
  }

}

//  Note: several more test audio files can be found here: 
//  https://github.com/schreibfaul1/ESP32-audioI2S/tree/master/additional_info/Testfiles
void connect(Audio *audio, int channel) {
  switch (channel){
  
    //  *** radio streams ***
    case 1:
    (*audio).connecttohost("http://rfcmedia2.streamguys1.com/thirdrock.mp3");   // aac
    break;

    // *** special test file to verify left and right channel stereo
    case 2:
    (*audio).connecttohost("https://github.com/pgiacalo/audio_test/raw/main/LeftRightCenterTest.mp3");        // mp3
    break;
      
    case 3:
    (*audio).connecttohost("http://www.wdr.de/wdrlive/media/einslive.m3u");     // m3u
    break;
  
    case 4:
    (*audio).connecttohost("http://s1.knixx.fm:5347/dein_webradio_vbr.opus");   // opus (ogg) 
    break;
  
    case 5:
    (*audio).connecttohost("http://eldoradolive02.akamaized.net/hls/live/2043453/eldorado/master.m3u8");   // HLS (ts)
    break;

    //*** web files ***
    case 6:
    (*audio).connecttohost("https://github.com/pgiacalo/audio_test/raw/main/sample.mp3");        // mp3
    break;
    
    case 7:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/Pink-Panther.wav");        // wav
    break;

    case 8:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/Olsen-Banden.mp3");        // mp3
    break;
  
    case 9:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/Miss-Marple.m4a");         // m4a (aac)
    break;
  
    case 10:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/sample.opus");             // opus 
    break;

    // //*** local files ***
  
    // case 19:
    // (*audio).connecttoFS(SD, "/test.wav");     // SD
    // break;
  
    // case 20:
    // (*audio).connecttoFS(SD_MMC, "/test.wav"); // SD_MMC
    // break;
  
    // case 21:
    // (*audio).connecttoFS(SPIFFS, "/test.wav"); // SPIFFS
    // break;

    // case 22:
    // (*audio).connecttospeech("Wenn die Hunde schlafen, kann der Wolf gut Schafe stehlen.", "de"); // Google TTS
    // break;
  }

}

void blinkSOS(int repeats){
  int tries = 0;
  while (tries < repeats){

      for (int i=0; i<3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
      }

      for (int i=0; i<3; i++) {
        delay(200);
        digitalWrite(LED_PIN, HIGH);
        delay(800);
        digitalWrite(LED_PIN, LOW);
      }

      for (int i=0; i<3; i++) {
        delay(200);
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
      }
    delay(1000);
    tries++;
  }
}

void setupButtons(){
  pinMode(CHAN_UP_PIN, INPUT_PULLUP);
  pinMode(CHAN_DOWN_PIN, INPUT_PULLUP);

  upButton.setDebounceTime(DEBOUNCE_TIME);
  downButton.setDebounceTime(DEBOUNCE_TIME);
}

void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("---------------------------------------------");
  Serial.println("------- Starting ESP32_I2S_WiFi_Radio -------");
  Serial.println("---------------------------------------------");
 
  pinMode(LED_PIN, OUTPUT); //use the LED to indicate WiFi status

  connectWiFi();
  Serial.println("------- WiFi Successfully Connected -------");
  
  setupAudio();
  Serial.println("------- Audio Setup Complete -------");
  
  setupButtons();
  Serial.println("------- setupButtons Complete -------");

  // currentChannelNumber = 1;
  Serial.println("Playing audio...");
  Serial.print("Playing Channel #");
  Serial.println(currentChannelNumber);
 
  connect(&audio, currentChannelNumber);

}

void loop() {
  int volume = map(analogRead(POT_PIN), 0, 4095, 0, VOLUME_CONTROL_STEPS);
  audio.setVolume(volume);

  bool changingChannels = false;

    upButton.loop();
   if ( upButton.isReleased() ) { 
    changingChannels = true;
    currentChannelNumber = currentChannelNumber + 1;
    if (currentChannelNumber > NUMBER_OF_CHANNELS){
      currentChannelNumber = 1;
    }
   }

    downButton.loop();
   if ( downButton.isReleased() ) { 
    changingChannels = true;
    currentChannelNumber = currentChannelNumber - 1;
    if (currentChannelNumber < 1){
      currentChannelNumber = NUMBER_OF_CHANNELS;
    }

   }

  if (changingChannels){
    Serial.print("Playing Channel #");
    Serial.println(currentChannelNumber);
    connect(&audio, currentChannelNumber);
  }

  audio.loop();
}
