#include <BleMouse.h>

BleMouse bleMouse("Mouse Touch IFSC");
const int limiarToque = 200; // Valor abaixo disso = Toque detectado

void setup() {
  Serial.begin(115200);
  bleMouse.begin();
}

void loop() {
  if (bleMouse.isConnected()) {
      Serial.println(touchRead(4));
      if (touchRead(4) < limiarToque) {
      bleMouse.click(MOUSE_LEFT);
      delay(300); // Delay para não clicar 1000x por segundo
    }
  }
}
