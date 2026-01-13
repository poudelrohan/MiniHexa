import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
sonar = Hiwonder_DEV.DEV_SONAR()
asr = Hiwonder_DEV.DEV_ASR()

while True:
    dis = sonar.getDistance()
    
    if dis < 15:
      r, g, b = 255, 0, 0
      sonar.setRGB(0, r, g, b)
      asr.speak(asr.ASR_ANNOUNCER, 5)
      time.sleep(1.5)
    else:
      r, g, b = 0, 255, 0
      sonar.setRGB(0, r, g, b)
    time.sleep_ms(100)

