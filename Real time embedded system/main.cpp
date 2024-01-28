#include <mbed.h>
//this file has all the functions for interacting
//with the screen
#include "drivers/LCD_DISCO_F429ZI.h"
#define BACKGROUND 1
#define FOREGROUND 0
#define GRAPH_PADDING 5

/*
Configure SPI:
    MOSI: PF_9
    MISO: PF_8
    SCLK: PF_7
    Chip Select: PC_1
*/ 
SPI spi(PF_9, PF_8, PF_7); // mosi, miso, sclk
DigitalOut cs(PC_1);


LCD_DISCO_F429ZI lcd;
//buffer for holding displayed text strings
char display_buf[2][60];
uint32_t graph_width=lcd.GetXSize()-2*GRAPH_PADDING;
uint32_t graph_height=graph_width;

//sets the background layer 
//to be visible, transparent, and
//resets its colors to all black
void setup_background_layer(){
  lcd.SelectLayer(BACKGROUND);
  lcd.Clear(LCD_COLOR_BLACK);
  lcd.SetBackColor(LCD_COLOR_BLACK);
  lcd.SetTextColor(LCD_COLOR_GREEN);
  lcd.SetLayerVisible(BACKGROUND,ENABLE);
  lcd.SetTransparency(BACKGROUND,0x7Fu);
}

//resets the foreground layer to
//all black
void setup_foreground_layer(){
    lcd.SelectLayer(FOREGROUND);
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_LIGHTGREEN);
}

//draws a rectangle with horizontal tick marks
//on the background layer. The spacing between tick
//marks in pixels is taken as a parameter
void draw_graph_window(uint32_t horiz_tick_spacing){
  lcd.SelectLayer(BACKGROUND);
  
  lcd.DrawRect(GRAPH_PADDING,GRAPH_PADDING,graph_width,graph_width);
  //draw the x-axis tick marks
  for (int32_t i = 0 ; i < graph_width;i+=horiz_tick_spacing){
    lcd.DrawVLine(GRAPH_PADDING+i,graph_height,GRAPH_PADDING);
  }
}

//maps inputY in the range minVal to maxVal, to a y-axis value pixel in the range
//minPixelY to MaxPixelY
uint16_t mapPixelY(float inputY,float minVal, float maxVal, int32_t minPixelY, int32_t maxPixelY){
  const float mapped_pixel_y=(float)maxPixelY-(inputY)/(maxVal-minVal)*((float)maxPixelY-(float)minPixelY);
  return mapped_pixel_y;
}

//DON'T COMMENT OUT THE ABOVE CODE





volatile int flag = 0;
int steps = 0;                // step walked
int state = 0;                //indicate the moving have started
double dist;                  // total accumulated distance
int16_t dataSet[400];         // distance travelled per 0.05s
double avg_dist_per_sec[40];  // distance travelled per sec
double avg_v_per_sec[40];     // velocity per sec


void setFlag();
int16_t readData(int addr);
void distCalculator(int16_t dataX, int16_t dataY, int16_t dataZ);

int main() {
  //LDC display
  setup_background_layer();

  setup_foreground_layer();

  //creates c-strings in the display buffers, in preparation
  //for displaying them on the screen
  // snprintf(display_buf[0],60,"width: %d pixels",lcd.GetXSize());
  // snprintf(display_buf[1],60,"height: %d pixels",lcd.GetYSize());
  lcd.SelectLayer(FOREGROUND);
  //display the buffered string on the screen
  lcd.DisplayStringAt(0, LINE(16), (uint8_t *)display_buf[1], LEFT_MODE);
  lcd.DisplayStringAt(0, LINE(17), (uint8_t *)display_buf[0], LEFT_MODE);
  
  //draw the graph window on the background layer
  // with x-axis tick marks every 10 pixels
  draw_graph_window(10);


  lcd.SelectLayer(FOREGROUND); 

  const float v_min = 0.0;
  const float v_max = 500.0;

  lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"time (i)", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"speed (v[i])", CENTER_MODE);
  uint32_t prev_x = GRAPH_PADDING;
  uint32_t prev_y_v = mapPixelY(0, v_min, v_max, GRAPH_PADDING, GRAPH_PADDING + graph_height);
  uint32_t prev_y_dist = mapPixelY(0, v_min, v_max, GRAPH_PADDING, GRAPH_PADDING + graph_height);



  
  // Deselect the chip
  cs=0;

  // set mode
  spi.write(0x20);
  spi.write(0xCF);
  cs=1;
  spi.format(8,3); 
  spi.frequency(100000);
	Ticker t;
  t.attach(&setFlag,0.05);

  // time variable used for calculated distance/velocity per second
  clock_t start_time = clock();
  clock_t last_time = start_time;
  double duration, interval_duration;
  int intervals_passed = 0;
  double distance_at_last_interval = 0;


  while(1) {
    duration = (double)(clock()-start_time) / CLOCKS_PER_SEC;
    interval_duration = (double)(clock()-last_time) / CLOCKS_PER_SEC;

    if (interval_duration >= 0.5){
      double average_distance = (dist - distance_at_last_interval) / interval_duration;
      avg_dist_per_sec[intervals_passed] = dist;
      avg_v_per_sec[intervals_passed] = average_distance;
      printf("%0.1fs : Accumulated average distance = %5f\n", intervals_passed*0.5+0.5, avg_dist_per_sec[intervals_passed]);
        printf("%0.1fs : Accumulated average velocity = %5f\n", intervals_passed*0.5+0.5, avg_v_per_sec[intervals_passed]);
      
      
      // 计算当前点的坐标
    uint32_t current_x = GRAPH_PADDING + 5*intervals_passed;
    double current_y_v = mapPixelY(avg_v_per_sec[intervals_passed]*50, v_min, v_max, GRAPH_PADDING, GRAPH_PADDING + graph_height);
    // 更新前一个点的坐标
    double current_y_dist= mapPixelY(avg_dist_per_sec[intervals_passed]*10, v_min, v_max, GRAPH_PADDING, GRAPH_PADDING + graph_height);


    // 绘制从前一个点到当前点的线
    lcd.SetTextColor(LCD_COLOR_RED);
    lcd.DrawLine(prev_x, prev_y_v, current_x, current_y_v);
    lcd.SetTextColor(LCD_COLOR_BLUE);
    lcd.DrawLine(prev_x, prev_y_dist, current_x, current_y_dist);


    prev_x = current_x;
    prev_y_v = current_y_v;
    prev_y_dist = current_y_dist;

    char tot_dist_str[30]; 

    // snprintf(display_buf[1],60,"height: %d pixels", tot_dist);
    snprintf(display_buf[1], sizeof(tot_dist_str), "Total Distance: %5f", dist);
    lcd.DisplayStringAt(0, LINE(17), (uint8_t *)display_buf[1], LEFT_MODE);
    
      
      last_time = clock();
      distance_at_last_interval = dist;
      intervals_passed++;
    }

    if (intervals_passed >= 40.0) break;

    if (flag) {
      int16_t dataX = readData(0xA8);
      int16_t dataY = readData(0xAA);
      int16_t dataZ = readData(0xAC);
      distCalculator(dataX, dataY, dataZ);
      flag = 0;
    }
    
  }
  return 0;
}

void setFlag() {          
  flag = 1;
}

// Measure gyro values from the angular velocity sensor
int16_t readData(int addr){
  cs = 0;
  spi.write(addr);
  uint8_t xl =spi.write(0x00);  // X_high
  cs = 1;
  cs = 0;
  spi.write(addr+1);  
  int8_t xh =spi.write(0x00);   // X_low
  cs = 1;
  return xh*256+xl;
}

void updateDistance() {
    const double stepLength = 0.7;  // Average step length in meters
    dist += stepLength / 10; // Increment a fraction of the step length

    // Limit the distance to the actual steps taken
    if (dist > steps * stepLength) {
        dist = steps * stepLength;
    }

    printf("The distance moved: %5.2f m\n", dist);
}

void distCalculator(int16_t dataX, int16_t dataY, int16_t dataZ){
  int16_t threshold_l[3] = {0, 0, -10000};
  int16_t threshold_h[3] = {0, 0, 10000};

  if(dataX < threshold_l[0] && dataY < threshold_l[1] && dataZ < threshold_l[2] && state == 0){
      state=1;
  }
  if(dataX > threshold_h[0] && dataY > threshold_h[1] && dataZ > threshold_l[2] && state == 1){
    state = 0;
    steps++;
  }
  updateDistance();
}
