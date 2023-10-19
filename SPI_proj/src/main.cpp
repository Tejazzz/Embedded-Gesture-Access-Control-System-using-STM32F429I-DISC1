/*This is a program designed for the STM32 F429ZI board, 
using the Mbed OS platform. The main purpose of the program is to 
read data from a gyroscope sensor, record a sequence of these data, 
and then compare subsequent sequences of data to the recorded sequence. 
If a subsequent sequence matches the recorded sequence closely enough, 
a digital output is set. The program also features an LCD display for message prompts.

Group 32
Syed Mohammed Ali Hussaini - sh6978
Talal Alsheqaih - ta2490
Tejas Hiremutt - th3038
Tianchen Gu - tg1749
*/
#include <mbed.h>
#include <chrono>
#include <numeric>
#include <cmath>
#include "drivers/LCD_DISCO_F429ZI.h"


using namespace std::chrono;   // Bring all symbols from std::chrono namespace into global namespace


SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs
InterruptIn int2(PA_2, PullDown);    // Instantiate interrupt input object for gyro data ready pin
InterruptIn userButton(PA_0);    // Instantiate interrupt input object for user button
DigitalOut recordLed(PG_13);     // Instantiate digital output object for record LED
DigitalOut unlockLed(PG_14);     // Instantiate digital output object for unlock LED
LCD_DISCO_F429ZI lcd;  // Instantiate LCD object

// Define constants
#define SEQUENCE_LENGTH 50  // Define the length of the recorded sequence
#define RECORDING_INTERVAL_MS 2000  // Define the interval for recording in milliseconds
#define TOLERANCE 1000  // Define the tolerance for sequence matching
#define OUT_X_L 0x28  // Register address for X-axis accelerometer data
#define OUT_Y_L 0x2A  // Register address for Y-axis accelerometer data
#define OUT_Z_L 0x2C  // Register address for Z-axis accelerometer data
#define CTRL_REG1 0x20  // Register address for control register 1
#define CTRL_REG1_CONFIG 0b01101111   // Configuration value for control register 1
#define CTRL_REG4 0x23   // Register address for control register 4
#define CTRL_REG4_CONFIG 0b00010000  // Configuration value for control register 4
#define CTRL_REG3 0x22   // Register address for control register 3
#define CTRL_REG3_CONFIG 0b00001000   // Configuration value for control register 3
#define SPI_FLAG 1   // Flag for SPI transfer completion
#define DATA_READY_FLAG 2   // Flag for data ready interrupt
#define BUTTON_PRESS_FLAG 4   // Flag for button press interrupt
#define NUM_AXES 3   // Number of axes for the accelerometer

// Declare arrays and variables
uint8_t write_buf[32];  // Buffer for SPI write data
uint8_t read_buf[32];   // Buffer for SPI read data
EventFlags flags;   // EventFlags object for synchronization
int16_t recordedSequence[NUM_AXES][SEQUENCE_LENGTH];   // Array to store the recorded sequence
int sequenceIndex = 0;   // Index for the current position in the sequence
bool recording = false;  // Flag to indicate if recording is in progress
bool hasRecordedSequence = false;   // Flag to indicate if a sequence has been recorded

// Callback function for SPI transfer completion
void spi_cb(int event)
{
   flags.set(SPI_FLAG);
};

// Callback function for data ready interrupt
void data_cb()
{
   flags.set(DATA_READY_FLAG);  // Set the SPI flag
};

// Callback function for button press interrupt
void buttonPressed_cb()
{
   if (!recording)
   {
       recording = true;   // Start recording
       recordLed = recording;   // Turn on the recording LED
       sequenceIndex = 0; // Reset sequence index before recording
   }
}

// Function to initialize SPI and gyro sensor
void init_spi()
{
   spi.format(8, 3);   // Set SPI format
   spi.frequency(1'000'000);   // Set SPI frequency

   // Write to control registers
   write_buf[0] = CTRL_REG1;
   write_buf[1] = CTRL_REG1_CONFIG;
   spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
   flags.wait_all(SPI_FLAG);


   write_buf[0] = CTRL_REG4;
   write_buf[1] = CTRL_REG4_CONFIG;
   spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
   flags.wait_all(SPI_FLAG);


   int2.rise(&data_cb);  // Attach data ready interrupt callback
   userButton.rise(callback(buttonPressed_cb));   // Attach button press interrupt callback


   write_buf[0] = CTRL_REG3; 
   write_buf[1] = CTRL_REG3_CONFIG;
   spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);   // Write to control register 3
   flags.wait_all(SPI_FLAG);   // Wait until the SPI transfer is complete


   if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1))  // Check if the data ready flag is not set and the data ready pin is high
   {
       flags.set(DATA_READY_FLAG);  // If so, set the data ready flag
   }
}

// Function to read gyro data
void read_gyro_data(int16_t *data)
{
   flags.wait_all(DATA_READY_FLAG);   // Wait until the data ready flag is set
   write_buf[0] = OUT_X_L | 0x80 | 0x40;    // Setup the address to read data
   spi.transfer(write_buf, 9, read_buf, 10, spi_cb, SPI_EVENT_COMPLETE);   // Start SPI transfer
   flags.wait_all(SPI_FLAG);  // Wait until the SPI transfer is complete
   // Convert received data to 16-bit integer
   data[0] = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
   data[1] = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
   data[2] = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);
}

// Function to calculate the mean of an array
double mean(int16_t data[], int size)
{
    double sum = std::accumulate(data, data + size, 0.0);  // Calculate the sum of the array
    return sum / size;   // Return the mean
}

// Function to calculate the standard deviation of an array
double stddev(int16_t data[], int size, double mean)
{
    double sum = 0.0;
    for(int i = 0; i < size; ++i)
        sum += (data[i] - mean) * (data[i] - mean);   // Sum of the squared differences
    return std::sqrt(sum / size); // Square root of the average squared difference
}

// Function to check if two sequences match
bool sequenceMatch(int16_t seq1[][SEQUENCE_LENGTH], int16_t seq2[][SEQUENCE_LENGTH])
{
   for (int axis = 0; axis < NUM_AXES; axis++)
   {
       double mean1 = mean(seq1[axis], SEQUENCE_LENGTH); // Calculate the mean of sequence 1
       double mean2 = mean(seq2[axis], SEQUENCE_LENGTH); // Calculate the mean of sequence 2
       double stddev1 = stddev(seq1[axis], SEQUENCE_LENGTH, mean1);  // Calculate the standard deviation of sequence 1
       double stddev2 = stddev(seq2[axis], SEQUENCE_LENGTH, mean2);  // Calculate the standard deviation of sequence 2

       // Check if the absolute differences between the means and standard deviations are within tolerance
       if (std::abs(mean1 - mean2) > TOLERANCE || std::abs(stddev1 - stddev2) > TOLERANCE)
       {
           return false;   // If not, return false
       }
   }
   return true;  // If all axes pass the check, return true
}

// Function to display a message on the LCD
void lcd_display_message(uint8_t **message, uint8_t numLines)  
{
    lcd.Clear(LCD_COLOR_WHITE);  // Clear the LCD screen
    // Loop through each line of the message
    for (uint8_t i = 0; i < numLines; i++) {
        // Display the line at the center of the LCD screen
        lcd.DisplayStringAt(0, LINE(5 + i), message[i], CENTER_MODE);
    }
}


int main()
{
   init_spi(); // Initialize SPI communication with the gyro sensor
   Timer recordingTimer;   // Create a timer object for controlling the recording duration
   lcd.Clear(LCD_COLOR_WHITE);   // Clear the LCD screen
   // Define messages to display on the LCD during various stages of the program
   uint8_t *startMessage[] = {(uint8_t *)"Move", (uint8_t *)"to", (uint8_t *)"record", (uint8_t *)"your", (uint8_t *)"Key"};
   uint8_t *stopMessage[] = {(uint8_t *)"Your", (uint8_t *)"key", (uint8_t *)"is", (uint8_t *)"recorded", (uint8_t *)"successfully"};
   uint8_t *unlockedMessage[] = {(uint8_t *)"Unlocked"};
   uint8_t *CompareMessage[] = {(uint8_t *)"Comparing..."};
   while (1)  // Infinite loop to keep the program running continuously
   {
       if (recording)  // Check if the system is in the recording mode
       {
           printf("Started recording.\n");  // Print a message to the console indicating the start of recording
           lcd_display_message(startMessage, 5);   // Display the start message on the LCD
           recordingTimer.start();   // Start the recording timer

           // Continue recording as long as the timer doesn't exceed the specified interval and the sequence isn't full
           while (recordingTimer.elapsed_time() < milliseconds(RECORDING_INTERVAL_MS) && sequenceIndex < SEQUENCE_LENGTH)
           {
               int16_t data[NUM_AXES];  // Array to store the gyro data
               read_gyro_data(data);   // Read the gyro data
               // Store the read gyro data in the recorded sequence
               for (int axis = 0; axis < NUM_AXES; axis++)
               {
                   recordedSequence[axis][sequenceIndex] = data[axis];
               }
               sequenceIndex++;  // Increment the sequence index


               ThisThread::sleep_for(50ms); // Sleep for a while to not overwhelm the sensor
           }


           recordingTimer.stop();   // Stop the recording timer
           recordingTimer.reset();  // Reset the recording timer


           printf("Stopped recording.\n");  // Print a message to the console indicating the end of recording
           lcd_display_message(stopMessage, 5);  // Display the stop message on the LCD


           recording = false;   // Set the recording status to false
           recordLed = recording;  // Turn off the record LED
           hasRecordedSequence = true; // Set the flag to true after recording


           // Add a 5 second delay after the recording mode
           ThisThread::sleep_for(5s);
       }
       else if (hasRecordedSequence) // Check if a sequence has been recorded
       {
           // Matching mode
           printf("Comparing...\n");  // Print a message to the console indicating the start of comparison
           lcd_display_message(CompareMessage, 1);  // Display the comparison message on the LCD
           int16_t currentSequence[NUM_AXES][SEQUENCE_LENGTH];  // Array to store the current sequence
           // Record a sequence for comparison
           for (int i = 0; i < SEQUENCE_LENGTH; i++)
           {
               int16_t data[NUM_AXES];  // Array to store the gyro data
               read_gyro_data(data);   // Read the gyro data
               // Store the read gyro data in the current sequence
               for (int axis = 0; axis < NUM_AXES; axis++)
               {
                   currentSequence[axis][i] = data[axis];  // The current sensor reading for the given axis is stored in the current sequence array.
               }
               ThisThread::sleep_for(50ms); // Sleep for a while to not overwhelm the sensor
           }

           // If the newly recorded sequence matches the original recorded sequence.
           if (sequenceMatch(recordedSequence, currentSequence))
           {
               lcd.Clear(LCD_COLOR_WHITE);  // The LCD display is cleared.
               printf("Sequence matched!\n");   // A message indicating that the sequences matched is printed to the console.
               lcd_display_message(unlockedMessage, 1);  // The "Unlocked" message is displayed on the LCD.
               unlockLed = 1;   // The unlock LED is turned on, indicating a successful match.
               ThisThread::sleep_for(8s);  // The program is put to sleep for 8 seconds to give the user time to view the "Unlocked" message and LED.
               unlockLed = 0;  // After the 8 second delay, the unlock LED is turned off.
           }
       }
   }
}
