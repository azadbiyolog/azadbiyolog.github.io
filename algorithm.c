/*
 * algorithm.c
 *
 *  Created on: Apr 12, 2025
 *      Author: azad
 */

 #include <main.h>
 #include <string.h>
 #include <math.h>
 #include "algorithm.h"
 #include <stdbool.h>
 
 
 volatile static uint8_t speed_level;
 float motorspeed_FrontRight;
 float motorspeed_FrontLeft;
 float motorspeed_RearRight;
 float motorspeed_RearLeft;
 
 //azad
 // Sensör ve motor konfigürasyonu
 
 #define NUM_SENSORS              8
 #define LEFT_SENSORS_START       0
 #define LEFT_SENSORS_END         3
 #define RIGHT_SENSORS_START      4
 #define RIGHT_SENSORS_END        7
 #define SINGLE_SENSOR_THRESHOLD  1
 #define MULTI_SENSOR_THRESHOLD   2
 
 // PID parametreleri
 //const float SENSOR_WEIGHTS[8] = {3.5f, 2.5f, 1.5f, 0.5f, -0.5f, -1.5f, -2.5f, -3.5f}; // Sol (+), Sağ (-)
 
 #define NUM_SENSORS              8
 #define SENSOR_SPACING_CM        2.5f    // Sensörler arası mesafe (cm)
 #define LINE_WIDTH_CM            2.0f    // Çizgi genişliği (cm)
 #define TURN_THRESHOLD           0.7f    // Dönüş için eşik değeri
 #define OBSTACLE_DETECTION_TIME  200     // Engel tespiti süresi (ms) önceki 500
 #define BLACK_THRESHOLD          1800    // Siyah zemin eşiği (ADC değeri)
 #define WHITE_THRESHOLD          800     // Beyaz zemin eşiği (ADC değeri)
 
 
 // Yumuşatma Parametreleri
 #define SMOOTHING_FACTOR      0.3f   // Düşük geçiş filtresi için
 #define ACCELERATION_RATE     0.02f  // Hız artış hassasiyeti önceki 0.20
 #define DEADZONE              0.05f  // Küçük hataları ignore et önceki 0.15
 
 
 // PID ve hız parametreleri
 const float SENSOR_WEIGHTS[8] = {-3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f};
 const float BASE_SPEED = 0.4f; //önceki 0.5
 const float MAX_TURN_SPEED = 0.6f; // Önceki 0.8'den düşürüldü
 const float MIN_SPEED = 0.3f;
 
 // Yumuşak geçiş için global değişkenler
 static float last_left_speed = BASE_SPEED;
 static float last_right_speed = BASE_SPEED;
 
 // Durum makinesi
 typedef enum {
     STATE_LINE_FOLLOWING,
     STATE_SHARP_TURN,
     STATE_OBSTACLE,
     // STATE_INTERSECTION kullanılmadığı için kaldırıldı
 } RobotState;
 
 // Yardımcı fonksiyon prototipi
 static inline float constrain_value(float val, float min, float max);
 
 volatile RobotState current_state = STATE_LINE_FOLLOWING;
 uint32_t last_obstacle_time = 0;
 
 // Yardımcı fonksiyon prototipi
 static inline float constrain_value(float val, float min, float max);
 static inline float low_pass_filter(float current_value, float last_value, float alpha);
 
 
 float custom_sqrt(float x) {
     if(x < 0) return 0;
     float guess = x;
     for(int i=0; i<10; i++) {
         guess = 0.5f * (guess + x/guess);
     }
     return guess;
 }
 //azad
 
 /* tanımlar, tip tanımları, parametreler
  * adc_result[] --> çizgi izleme sensor data
  * encoder_count_front_right, encoder_count_front_left; encoder_count_rear_right, encoder_count_rear_left;
  */
 
 //fonksiyonlar
 void init_application(void){
     //motor stop
     motorspeed_FrontRight =0.50;
     motorspeed_FrontLeft =0.50;
     motorspeed_RearRight =0.50;
     motorspeed_RearLeft =0.50;
 
     motorForward(motorspeed_FrontRight, motorspeed_FrontLeft, motorspeed_RearRight, motorspeed_RearLeft);
 
     //parametre başlangıç değer atama
     PID_init();
 
 }
 /*
 // Çizgi pozisyon hesaplama
 //int32_t calculate_line_position(volatile uint16_t *sensor_values) {
     const int32_t sensor_weights[ADC_CHANNEL_COUNT] = {-35, -25, -15, -5, 5, 15, 25, 35};
     int32_t weighted_sum = 0;
     int32_t sum = 0;
 
     for(int i=0; i<ADC_CHANNEL_COUNT; i++) {
         weighted_sum += sensor_values[i] * sensor_weights[i];
         sum += sensor_values[i];
     }
 
     return (sum == 0) ? 0 : (weighted_sum * 100) / sum;
 }
 */
 /*
 void run_application (void){
 
     if (pid_calculate){
         pid_calculate = 0;
         // myPID[FRONT_RIGHT].Set_point=0;
         motorspeed_FrontRight = PID_Calculation(encoder_count_front_right, &myPID[FRONT_RIGHT])/100;
         motorspeed_FrontLeft = PID_Calculation(encoder_count_front_left, &myPID[FRONT_LEFT])/100;
         motorspeed_RearRight = PID_Calculation(encoder_count_rear_right, &myPID[REAR_RIGHT])/100;
         motorspeed_RearLeft = PID_Calculation(encoder_count_rear_left, &myPID[REAR_LEFT])/100;
 
         //update speed
         motorForward(motorspeed_FrontRight, motorspeed_FrontLeft, motorspeed_RearRight, motorspeed_RearLeft);
     }
 }
 
 */
 
 // akşam hazırlandı
 
 void run_application(void) {
     if (!pid_calculate) return;
     pid_calculate = 0;
 
     // Sensör verilerini normalize et ve pozisyonu hesapla
     float weighted_sum = 0.0f;
     float sensor_sum = 0.0f;
     float normalized;
     float position = 0.0f;
 
     // Sensör sıralı okuma analizi için işaret dizisi
     int active_indices[NUM_SENSORS];
     int active_count = 0;
 
     for (uint8_t i = 0; i < NUM_SENSORS; i++) {
         normalized = (float)(adc_result[i] - 800) / (4095.0f - 800.0f);
         normalized = constrain_value(normalized, 0.0f, 1.0f);
         if (normalized > 0.2f) {  // siyah okuma olarak kabul edilecek minimum değer
             active_indices[active_count++] = i;
         }
         weighted_sum += SENSOR_WEIGHTS[i] * normalized;
         sensor_sum += normalized;
     }
     if (sensor_sum > 0.0f) {
         position = weighted_sum / sensor_sum;
     }
 
     // Sağ ve sol sensörlerden kaç tanesi siyah görüyor?
     int left_black = 0, right_black = 0;
     for (int i = 0; i < 4; i++) {
         if (adc_result[i] > BLACK_THRESHOLD) left_black++;
     }
     for (int i = 4; i < 8; i++) {
         if (adc_result[i] > BLACK_THRESHOLD) right_black++;
     }
 
    // Dönüş manevrası: yalnızca tek bir tarafta sensör aktifse ve sıralı artış/azalış varsa
 //    if ((left_black == 1 && right_black == 0) || (right_black == 1 && left_black == 0)) {
 //        if (active_count >= 2) {
 //            // Sıralı artan index: sağa dönüş, azalan index: sola dönüş
 //            if (active_indices[0] < active_indices[1]) {
 //                position = 2.0f;  // sağa dönüş düzeltmesi
 //            } else {
 //                position = -2.0f; // sola dönüş düzeltmesi
 //          }
 //      }
 //  }
 
   // Dönüş manevrası - gelişmiş eğim analizi
     if ((left_black == 1 && right_black == 0) || (right_black == 1 && left_black == 0)) {
         float turn_weight = 0.0f;
         float turn_sum = 0.0f;
         for (int i = 0; i < active_count; i++) {
             float weight = SENSOR_WEIGHTS[active_indices[i]];
             float strength = (float)(adc_result[active_indices[i]] - 800) / (4095.0f - 800.0f);
             strength = constrain_value(strength, 0.0f, 1.0f);
             turn_weight += weight * strength;
             turn_sum += strength;
         }
         if (turn_sum > 0.0f) {
             position = turn_weight / turn_sum;
         }
     }
 
 
     // Özel durum: bir tarafta 1 sensör, diğer tarafta >=2 sensör varsa düz git
     if ((left_black == 1 && right_black >= 2) || (right_black == 1 && left_black >= 2)) {
         position = 0.0f;
     }
 
     // PID ile pozisyon düzeltmesini hesapla (line PID kullanılmıyor, motor PID'leriyle doğrudan entegre)
     float correction = position * 0.3f;  // çizgi pozisyonu -3.5 → +3.5 arası, düzeltme oranı ayarlanabilir
 
     float base_FL = BASE_SPEED + correction;
     float base_FR = BASE_SPEED - correction;
     float base_RL = BASE_SPEED + correction;
     float base_RR = BASE_SPEED - correction;
 
     // Set_point'leri encoder bazlı PID sistemine gönder (senin PID sistemin kullanılıyor)
     myPID[FRONT_LEFT].Set_point  = constrain_value(base_FL * 20.0f, 15.0f, 35.0f);
     myPID[FRONT_RIGHT].Set_point = constrain_value(base_FR * 20.0f, 15.0f, 35.0f);
     myPID[REAR_LEFT].Set_point   = constrain_value(base_RL * 20.0f, 15.0f, 35.0f);
     myPID[REAR_RIGHT].Set_point  = constrain_value(base_RR * 20.0f, 15.0f, 35.0f);
 
     // Encoder'lara göre motor hızlarını PID ile hesapla
     float speed_FL = PID_Calculation(encoder_count_front_left,  &myPID[FRONT_LEFT])  / 100.0f;
     float speed_FR = PID_Calculation(encoder_count_front_right, &myPID[FRONT_RIGHT]) / 100.0f;
     float speed_RL = PID_Calculation(encoder_count_rear_left,   &myPID[REAR_LEFT])   / 100.0f;
     float speed_RR = PID_Calculation(encoder_count_rear_right,  &myPID[REAR_RIGHT])  / 100.0f;
 
     // Hızları minimum ve maksimum değerlerle sınırlayalım
     speed_FL = constrain_value(speed_FL, MIN_SPEED, MAX_TURN_SPEED);
     speed_FR = constrain_value(speed_FR, MIN_SPEED, MAX_TURN_SPEED);
     speed_RL = constrain_value(speed_RL, MIN_SPEED, MAX_TURN_SPEED);
     speed_RR = constrain_value(speed_RR, MIN_SPEED, MAX_TURN_SPEED);
 
     // Motorlara hızları gönder
     motorForward(speed_FR, speed_FL, speed_RR, speed_RL);
 }
 
 
 /*
 void run_application(void) {
      if (!pid_calculate) return;
      pid_calculate = 0;
 
      // Sensör verilerini normalize et ve pozisyonu hesapla
      float weighted_sum = 0.0f;
      float sensor_sum = 0.0f;
      float normalized;
      float position = 0.0f;
 
      // Sensör sıralı okuma analizi için işaret dizisi
      int active_indices[NUM_SENSORS];
      int active_count = 0;
 
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          normalized = (float)(adc_result[i] - 800) / (4095.0f - 800.0f);
          normalized = constrain_value(normalized, 0.0f, 1.0f);
          if (normalized > 0.2f) {  // siyah okuma olarak kabul edilecek minimum değer
              active_indices[active_count++] = i;
          }
          weighted_sum += SENSOR_WEIGHTS[i] * normalized;
          sensor_sum += normalized;
      }
      if (sensor_sum > 0.0f) {
          position = weighted_sum / sensor_sum;
      }
 
 //      // Sağ ve sol sensörlerden kaç tanesi siyah görüyor?
 //      int left_black = 0, right_black = 0;
 //      for (int i = 0; i < 4; i++) {
 //          if (adc_result[i] > BLACK_THRESHOLD) left_black++;
 //      }
 //      for (int i = 4; i < 8; i++) {
 //          if (adc_result[i] > BLACK_THRESHOLD) right_black++;
 //      }
 
 //    // Dönüş manevrası
 //      if ((left_black == 1 && right_black == 0) || (right_black == 1 && left_black == 0)) {
 //          float turn_weight = 0.0f;
 //          float turn_sum = 0.0f;
 //          for (int i = 0; i < active_count; i++) {
 //              float weight = SENSOR_WEIGHTS[active_indices[i]];
 //              float strength = (float)(adc_result[active_indices[i]] - 800) / (4095.0f - 800.0f);
 //              strength = constrain_value(strength, 0.0f, 1.0f);
 //              turn_weight += weight * strength;
 //              turn_sum += strength;
 //          }
 //          if (turn_sum > 0.0f) {
 //              position = turn_weight / turn_sum;
 //          }
 //      }
 
 
 //      // Özel durum: bir tarafta 1 sensör, diğer tarafta >=2 sensör varsa düz git
 //      if ((left_black == 1 && right_black >= 2) || (right_black == 1 && left_black >= 2)) {
 //          position = 0.0f;
 //      }
 
      // PID ile pozisyon düzeltmesini hesapla (line PID kullanılmıyor, motor PID'leriyle doğrudan entegre)
      float correction = position * 0.3f;  // çizgi pozisyonu -3.5 → +3.5 arası, düzeltme oranı ayarlanabilir
 
      float base_FL = BASE_SPEED + correction;
      float base_FR = BASE_SPEED - correction;
      float base_RL = BASE_SPEED + correction;
      float base_RR = BASE_SPEED - correction;
 
     //  // Set_point'leri encoder bazlı PID sistemine gönder (senin PID sistemin kullanılıyor)
     //  myPID[FRONT_LEFT].Set_point  = constrain_value(base_FL * 20.0f, 15.0f, 35.0f);
     //  myPID[FRONT_RIGHT].Set_point = constrain_value(base_FR * 20.0f, 15.0f, 35.0f);
     //  myPID[REAR_LEFT].Set_point   = constrain_value(base_RL * 20.0f, 15.0f, 35.0f);
     //  myPID[REAR_RIGHT].Set_point  = constrain_value(base_RR * 20.0f, 15.0f, 35.0f);
 
     //  // Encoder'lara göre motor hızlarını PID ile hesapla
     //  float speed_FL = PID_Calculation(encoder_count_front_left,  &myPID[FRONT_LEFT])  / 100.0f;
     //  float speed_FR = PID_Calculation(encoder_count_front_right, &myPID[FRONT_RIGHT]) / 100.0f;
     //  float speed_RL = PID_Calculation(encoder_count_rear_left,   &myPID[REAR_LEFT])   / 100.0f;
     //  float speed_RR = PID_Calculation(encoder_count_rear_right,  &myPID[REAR_RIGHT])  / 100.0f;
 
     //  // Hızları minimum ve maksimum değerlerle sınırlayalım
     //  speed_FL = constrain_value(speed_FL, MIN_SPEED, MAX_TURN_SPEED);
     //  speed_FR = constrain_value(speed_FR, MIN_SPEED, MAX_TURN_SPEED);
     //  speed_RL = constrain_value(speed_RL, MIN_SPEED, MAX_TURN_SPEED);
     //  speed_RR = constrain_value(speed_RR, MIN_SPEED, MAX_TURN_SPEED);
 
      // Motorlara hızları gönder
      motorForward(base_FR, base_FL, base_RR, base_RL);
  }
 */
 static inline float constrain_value(float val, float min, float max) {
     return (val < min) ? min : ((val > max) ? max : val);
 }
 
 static inline float low_pass_filter(float current_value, float last_value, float alpha) {
     return (alpha * current_value) + ((1.0f - alpha) * last_value);
 }
 
 
 //deepseek
 
 /*
 void run_application(void) {
     static int stabilization_counter = 0;
 
     if(pid_calculate) {
         pid_calculate = 0;
 
         // 1. Sensör Okuma ve Analiz
         int left_active = 0, right_active = 0;
         for(int i=0; i<8; i++) {
             bool active = (adc_result[i] > BLACK_THRESHOLD);
             if(i <= 3) left_active += active;
             else right_active += active;
         }
 
         // 2. Özel Durum: 1 vs >1 Sensör
         if((left_active == 1 && right_active > 1) || (right_active == 1 && left_active > 1)) {
             // Tüm motorlara AYNI SET_POINT ataması
             myPID[FRONT_RIGHT].Set_point = 50.0f;
             myPID[FRONT_LEFT].Set_point = 50.0f;
             myPID[REAR_RIGHT].Set_point = 50.0f;
             myPID[REAR_LEFT].Set_point = 50.0f;
 
             // Encoder değerlerine göre PID çıkışı
             float fr = PID_Calculation(encoder_count_front_right, &myPID[FRONT_RIGHT]) / 100.0f;
             float fl = PID_Calculation(encoder_count_front_left, &myPID[FRONT_LEFT]) / 100.0f;
             float rr = PID_Calculation(encoder_count_rear_right, &myPID[REAR_RIGHT]) / 100.0f;
             float rl = PID_Calculation(encoder_count_rear_left, &myPID[REAR_LEFT]) / 100.0f;
 
             motorForward(fr, fl, rr, rl);
             stabilization_counter = 0;
             return;
         }
 
         // 3. Çizgi Kaybı Durumu (Tüm sensörler beyaz)
         if(left_active == 0 && right_active == 0) {
             // Son bilinen pozisyona göre dönüş yap
             if(last_known_position > LOST_LINE_THRESHOLD) {
                 // Son pozisyon sağda ise sola dön
                 motorForward(0.6f, 0.3f, 0.6f, 0.3f);
             } else if(last_known_position < -LOST_LINE_THRESHOLD) {
                 // Son pozisyon solda ise sağa dön
                 motorForward(0.3f, 0.6f, 0.3f, 0.6f);
             } else {
                 // Son pozisyon belirsizse sağa dön (default)
                 motorForward(0.4f, 0.6f, 0.4f, 0.6f);
             }
             return;
         }
 
         // 4. Normal PID Kontrolü
         float weights[] = {3.0f, 2.0f, 1.0f, 0.5f, -0.5f, -1.0f, -2.0f, -3.0f};
         float position = 0.0f, total = 0.0f;
 
         for(int i=0; i<8; i++) {
             position += weights[i] * sensor_values[i];
             total += sensor_values[i];
         }
 
         if(total > 0) position /= total;
         last_known_position = position; // Pozisyonu güncelle
 
         // PID Hesaplama
         float error = position;
         static float integral = 0.0f;
         static float prev_error = 0.0f;
 
         integral += error * myPID[FRONT_RIGHT].Ki;
         integral = constrain_value(integral, -1.0f, 1.0f);
 
         float correction = (myPID[FRONT_RIGHT].Kp * error) +
                           integral +
                           (myPID[FRONT_RIGHT].Kd * (error - prev_error)/myPID[FRONT_RIGHT].Ts;
 
         prev_error = error;
 
         // 5. Motor Hızları
         float base = 0.5f;
         float left = constrain_value(base - correction, 0.3f, 0.7f);
         float right = constrain_value(base + correction, 0.3f, 0.7f);
 
         motorForward(right, left, right, left);
     }
 }
 
 
 */
 //PİD algoritması
 
 void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
 {
     //user button -PE0- ExtInt with RisingEdge - PullDown
     if (GPIO_Pin == User_Button_Pin) {
         //User button pressed
         HAL_GPIO_TogglePin(User_LED2_GPIO_Port, User_LED2_Pin);
 
         if (++speed_level > 4)
             speed_level = 0;
         if (speed_level == 1){
             myPID[FRONT_RIGHT].Set_point = 45.00;
         }
         else if (speed_level == 2){
             myPID[FRONT_RIGHT].Set_point = 75.00;
         }
         else if (speed_level == 3){
             myPID[FRONT_RIGHT].Set_point = 100.00;
         }
         else if (speed_level == 4){
             myPID[FRONT_RIGHT].Set_point = 125.00;
         }
         else {
             myPID[FRONT_RIGHT].Set_point = 30.00;
 
         }
     }
 
 }
 
 
 