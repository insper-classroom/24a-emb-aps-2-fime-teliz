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
        if data:
            if data == b'\x01':
                data = ser.read(2)
                value = int.from_bytes(data, byteorder='big', signed=True)
                value *= 21.3
                print(f"X: {value}")
                #pyautogui.move(value, 0)
            if data == b'\x02':
                data = ser.read(1) 
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 1:
                    pyautogui.mouseDown()
                elif value == 0:
                    pyautogui.mouseUp()
            if data == b'\x03':
                data = ser.read(1)
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 1:
                    data = ser.read()
                    value = int.from_bytes(data, byteorder='big', signed=True)
                    if value == 2:    
                        pyautogui.keyDown('d')
                        pyautogui.keyUp('a')
                    elif value == -2:
                        pyautogui.keyDown('a')
                        pyautogui.keyUp('d')
                    elif value == 0:
                        pyautogui.keyUp('d')
                        pyautogui.keyUp('a')
                elif value == 0:
                    data = ser.read(1)
                    value = int.from_bytes(data, byteorder='big', signed=True)
                    if value == 2:
                        pyautogui.keyDown('w')
                        pyautogui.keyUp('s')
                    elif value == -2:
                        pyautogui.keyDown('s')
                        pyautogui.keyUp('w')
                    elif value == 0:
                        pyautogui.keyUp('w')
                        pyautogui.keyUp('s')

except KeyboardInterrupt:
    print("Program terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()