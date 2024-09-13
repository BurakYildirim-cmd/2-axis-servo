import cv2
import numpy as np
import time
from sys import argv
import RPi.GPIO as GPIO
from time import sleep
from random import randint

# PID parametreleri
Kp = 0.5  # Oransal kazanç
Ki = 0.1  # İntegral kazanç
Kd = 0.05 # Türevsel kazanç

# PID denetleyicisi değişkenleri
previous_error_x = 0
integral_x = 0
previous_error_y = 0
integral_y = 0

GPIO.setmode(GPIO.BOARD)

# X ekseni servo kontrolü için GPIO 11
GPIO.setup(11, GPIO.OUT)
servo_x = GPIO.PWM(11, 50)

# Y ekseni servo kontrolü için GPIO 13
GPIO.setup(13, GPIO.OUT)
servo_y = GPIO.PWM(13, 50)

# Röle kontrolü için GPIO 15
GPIO.setup(15, GPIO.OUT)
GPIO.output(15, GPIO.LOW)  # Röleyi kapalı olarak başlat

servoDegerX = 105
servoDegerY = 105

def servoBaslat():
    # Servoları başlat
    DutyCycleX = 1/18 * (servoDegerX) + 2
    DutyCycleY = 1/18 * (servoDegerY) + 2
    servo_x.start(DutyCycleX)
    servo_y.start(DutyCycleY)
    sleep(0.05)

def servoOynat(servo, deger):
    # Verilen servo motoru belirtilen değere ayarla
    DutyCycle = 1/18 * (deger) + 2
    servo.ChangeDutyCycle(DutyCycle)
    sleep(0.05)

def pid_control(error, dt, previous_error, integral):
    # PID hesaplamaları
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output, previous_error, integral

def roleyi_ac_kapat():
    # Röleyi aç ve 2 saniye bekle, sonra kapa
    GPIO.output(15, GPIO.HIGH)  # Röleyi aç
    print("Röle açıldı!")
    sleep(2)  # 2 saniye bekle
    GPIO.output(15, GPIO.LOW)   # Röleyi kapa
    print("Röle kapandı!")

# Kamerayı başlat
cameraWidth  = float(argv[2])
cameraHeight = float(argv[3])
cameraIndex  = int(argv[4])

video_capture = cv2.VideoCapture(cameraIndex)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, cameraWidth)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, cameraHeight)

# Zaman kontrolü için başlatma
prev_time = time.time()

servoBaslat()  # Servoları başlat

while True:
    if not video_capture.isOpened():
        print('Kamera yuklenemedi!')
        sleep(3)
        pass

    ret, frame = video_capture.read()
    frame = cv2.flip(frame, 1)

    # HSV renk uzayına dönüştürme
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Kırmızı renk aralıkları tanımlanır (alt ve üst sınır)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    # Kırmızı renk maskesi oluşturulur
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Kontur bulma
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # En büyük konturu seç
        largest_contour = max(contours, key=cv2.contourArea)
        (x, y, w, h) = cv2.boundingRect(largest_contour)
        
        # Kırmızı nesnenin merkezi
        object_center_x = x + w // 2
        object_center_y = y + h // 2
        
        # Kare boyutu
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2
        
        # Hata hesaplama (merkezden sapma)
        error_x = frame_center_x - object_center_x
        error_y = frame_center_y - object_center_y
        
        # Zaman farkı (dt) hesaplama
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        
        # PID kontrolü ile çıktıları hesapla
        pid_output_x, previous_error_x, integral_x = pid_control(error_x, dt, previous_error_x, integral_x)
        pid_output_y, previous_error_y, integral_y = pid_control(error_y, dt, previous_error_y, integral_y)
        
        # Sonuçları ekrana yazdır
        print(f"Error X: {error_x}, PID Output X: {pid_output_x}")
        print(f"Error Y: {error_y}, PID Output Y: {pid_output_y}")
        
        # Algılanan nesneye dikdörtgen çiz
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # Merkezine bir nokta çiz
        cv2.circle(frame, (object_center_x, object_center_y), 5, (255, 0, 0), -1)

        # Röleyi aç/kapat
        roleyi_ac_kapat()

        # Servo kontrolü için nesnenin konumunu kullan
        # X ekseni kontrolü (sağ-sol)
        if object_center_x < frame_center_x - 40:  # Nesne sol tarafta
            if servoDegerX > 10:
                servoDegerX -= 3
            servoOynat(servo_x, servoDegerX)
        elif object_center_x > frame_center_x + 40:  # Nesne sağ tarafta
            if servoDegerX < 160:
                servoDegerX += 3
            servoOynat(servo_x, servoDegerX)

        # Y ekseni kontrolü (yukarı-aşağı)
        if object_center_y < frame_center_y - 40:  # Nesne yukarıda
            if servoDegerY > 10:
                servoDegerY -= 3
            servoOynat(servo_y, servoDegerY)
        elif object_center_y > frame_center_y + 40:  # Nesne aşağıda
            if servoDegerY < 160:
                servoDegerY += 3
            servoOynat(servo_y, servoDegerY)

    # Görüntüyü göster
    cv2.imshow('Kamera', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Temizle ve çık
servo_x.stop()
servo_y.stop()
GPIO.cleanup()
video_capture.release()
cv2.destroyAllWindows()
