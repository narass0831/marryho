import spidev
import time

# SPI 설정
spi = spidev.SpiDev()
spi.open(0, 0)  # 버스 0, 디바이스(SS) 0번
spi.max_speed_hz = 50000  # SPI 통신 속도 설정

def send_data(data):
    # 1바이트씩 전송
    spi.xfer2([data])
    print(f"Sent: {data}")

try:
    while True:
        # 0~255 범위의 임의 데이터 전송
        send_data(0xA5)  # 예: 0xA5 전송
        time.sleep(1)

except KeyboardInterrupt:
    spi.close()
    print("SPI 통신 종료")
