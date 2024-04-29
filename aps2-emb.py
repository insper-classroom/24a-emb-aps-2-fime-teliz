import serial
import pyautogui

ser = serial.Serial('COM11', 115200)
stateup = 1

try:
    pyautogui.PAUSE = 0
    # sync package
    while True:
        # print('Waiting for sync package...')
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
                #print(f"X: {value}")
                #pyautogui.move(value, 0)
            if data == b'\x02':
                data = ser.read(1) 
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 1 and stateup == 1:
                    pyautogui.mouseDown()
                    print(f"click{value}")
                    state = 0
                elif value == 0:
                    pyautogui.mouseUp()
                    state = 1
                    print(f"soltou click{value}")
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
            if data == b'\x04':
                data = ser.read(1)
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 1:
                    pyautogui.mouseDown(button='right')
                else:
                    pyautogui.mouseUp(button='right')

            if data == b'\x05':
                data = ser.read(1)
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 1:
                    pyautogui.mouseDown()
                else:
                    pyautogui.mouseUp()
            if data == b'\x06':
                data = ser.read(1)
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 1:
                    pyautogui.scroll(1)
                elif value == 2:
                    pyautogui.scroll(-1)


                

except KeyboardInterrupt:
    print("Program terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()