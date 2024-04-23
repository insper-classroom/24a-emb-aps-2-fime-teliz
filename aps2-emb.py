import serial
import pyautogui

ser = serial.Serial('COM8', 115200)


try:
    # sync package
    while True:
        print('Waiting for sync package...')
        while True:
            data = ser.read(1)
            if data == b'\xff':
                break

        #Read 4 bytes from UART
        data = ser.read(1)
        if data == b'\x01':
            data = ser.read(2)
            value = int.from_bytes(data, byteorder='big', signed=True)
            #continuar aqui que ta incompleto

        if data == b'\x02':
            data = ser.read(1) 
            pyautogui.mouseDown(button='left')


except KeyboardInterrupt:
    print("Program terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()