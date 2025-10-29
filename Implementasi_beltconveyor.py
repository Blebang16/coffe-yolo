import cv2
import time
import RPi.GPIO as GPIO
from ultralytics import YOLO

#konfigurasi
CONF_THRESHOLD = 0.5        #batas keyakinan deteksi
SCAN_CAMERA_MAX = 5         #maksimall kamera yang akan di scan

SERVO1_PIN = 23
SERVO2_PIN = 24

NETRAL_POS = 90
SORTIR_POS = 0

#setup gpio & servo
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SERVO1_PIN, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)

pwm1 = GPIO.PWM(SERVO1_PIN, 50)
pwm2 = GPIO.PWM(SERVO2_PIN, 50)
pwm1.start(0)
pwm2.start(0)

def set_angle(pwm, angle):
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)
    pwm.ChangeDutyCycle(0)

#reset posisi awal servo
set_angle(pwm1, NETRAL_POS)
set_angle(pwm2, NETRAL_POS)
print("Servo reset ke posisi netral")

#uoad model yolo
model = YOLO("/home/stich/Desktop/coding/kaggle/working/runs/buahkopie100b8/weights/best.pt")

#pindai kamera
available_cams = []
for i in range(SCAN_CAMERA_MAX):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f" Kamera ditemukan di index {i}")
        available_cams.append(i)
        cap.release()

if len(available_cams) < 2:
    print(" Dibutuhkan 2 kamera, tapi hanya ditemukan:", len(available_cams))
    exit()

cap1 = cv2.VideoCapture(available_cams[0])
cap2 = cv2.VideoCapture(available_cams[1])
print(f" Menggunakan kamera index {available_cams[0]} dan {available_cams[1]}")

def read_frame(cap):
    for _ in range(3):  #buang beberapa frame awal agar lebih stabil
        cap.read()
    ret, frame = cap.read()
    return ret, frame

#fungsi servo aksi
def servo_action(pwm, name, delay_time, hold_time=1):

    print(f" {name}: Tunggu {delay_time}s sebelum sortir...")
    time.sleep(delay_time)
    print(f" {name}: SORTIR ke {SORTIR_POS}° (tahan {hold_time}s)")
    set_angle(pwm, SORTIR_POS)
    time.sleep(hold_time)
    print(f" {name}: Kembali ke NETRAL {NETRAL_POS}°")
    set_angle(pwm, NETRAL_POS)


#perulangan utama

last_action_time = {"servo1": 0, "servo2": 0}
action_cooldown = 2  #cooldown agar servo tidak spam

try:
    while True:
        ret1, frame1 = read_frame(cap1)
        ret2, frame2 = read_frame(cap2)
        if not ret1 or not ret2:
            print("Kamera gagal baca frame")
            continue

        #jalankan deteksi yolo
        results1 = model(frame1, verbose=False)[0]
        results2 = model(frame2, verbose=False)[0]

        #ambil kelas yang terdeteksi di masing-masing kamera
        detected_classes1 = [
            model.names[int(cls)]
            for cls, conf in zip(results1.boxes.cls, results1.boxes.conf)
            if conf > CONF_THRESHOLD
        ]
        detected_classes2 = [
            model.names[int(cls)]
            for cls, conf in zip(results2.boxes.cls, results2.boxes.conf)
            if conf > CONF_THRESHOLD
        ]

        print(" Kamera1:", detected_classes1, "| Kamera2:", detected_classes2)

        #gambar boundingbox di setiap kamera
        annotated_frame1 = results1.plot()
        annotated_frame2 = results2.plot()

        now = time.time()

        #logika baru: servo bergerak jika KEDUA kamera mendeteksi kelas yang sama
        
        #jika kedua kamera mendeteksi kelas 'a' (misal: buah matang)
        if "a" in detected_classes1 and "a" in detected_classes2:
            if now - last_action_time["servo1"] > action_cooldown:
                servo_action(pwm1, "Servo1 (Matang)", delay_time=1, hold_time=1)
                last_action_time["servo1"] = time.time()

        #jika kedua kamera mendeteksi kelas 'b' (misal: buah belum matang)
        elif "b" in detected_classes1 and "b" in detected_classes2:
            if now - last_action_time["servo2"] > action_cooldown:
                servo_action(pwm2, "Servo2 (Belum Matang)", delay_time=4, hold_time=1)
                last_action_time["servo2"] = time.time()

        #tampilkan hasil kamera
        cv2.imshow("Kamera 1", annotated_frame1)
        cv2.imshow("Kamera 2", annotated_frame2)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\ Program dihentikan manual")

finally:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()
    print("GPIO & Kamera sudah dibersihkan")