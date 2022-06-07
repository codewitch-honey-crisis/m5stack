#include <Arduino.h>
#include <SPIFFS.h>
#include <SD.h>
#include <tft_io.hpp>
#include <m5stack.hpp>
#include <gfx_cpp14.hpp>
#include "telegrama.hpp"
using namespace gfx;
using namespace arduino;

m5stack m5;
using color_t = color<typename m5stack::pixel_type>;

void setup() {
    Serial.begin(115200);
    m5.fill(m5.bounds(),color_t::white);
    draw::text(m5,m5.bounds(),spoint16::zero(),"test m5stack",Telegrama_otf,Telegrama_otf.scale(20),color_t::purple);
    if(!m5.initialize()) {
        Serial.printf("Unable to initialize M5 Stack\n");
        while(1);
    }
}
void loop() {
    Serial.printf("temp: %fF\n",m5.spatial().temp());
    float p,r,y;
    m5.spatial().gyro(&p,&r,&y);
    Serial.printf("pitch: %f, roll: %f, yaw: %f\n",p,r,y);
    delay(1000);
}