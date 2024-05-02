import serial
import pydirectinput
import pyautogui

stateup = 1
ser = serial.Serial('COM11', 9600)
try:
    pydirectinput.PAUSE = 0
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
                print(f"X: {value}")
                value *=-10
                pydirectinput.move(value, 0)
            if data == b'\x02':
                data = ser.read(1) 
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 1 and stateup == 1:
                    pydirectinput.mouseDown()
                    print(f"click{value}")
                    stateup = 0
                elif value == 0:
                    pydirectinput.mouseUp()
                    stateup = 1
            if data == b'\x03':
                data = ser.read(1)
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 0:
                    data = ser.read()
                    value = int.from_bytes(data, byteorder='big', signed=True)
                    if value == 2:    
                        pydirectinput.keyDown('d')
                        pydirectinput.keyUp('a')
                    elif value == -2:
                        pydirectinput.keyDown('a')
                        pydirectinput.keyUp('d')
                    elif value == 0:
                        pydirectinput.keyUp('d')
                        pydirectinput.keyUp('a')
                elif value == 1:
                    data = ser.read(1)
                    value = int.from_bytes(data, byteorder='big', signed=True)
                    if value == 2:
                        pydirectinput.keyDown('w')
                        pydirectinput.keyUp('s')
                    elif value == -2:
                        pydirectinput.keyDown('s')
                        pydirectinput.keyUp('w')
                    elif value == 0:
                        pydirectinput.keyUp('w')
                        pydirectinput.keyUp('s')
            if data == b'\x04':
                data = ser.read(1)
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 1:
                    pydirectinput.mouseDown(button='right')
                else:
                    pydirectinput.mouseUp(button='right')

            if data == b'\x05':
                data = ser.read(1)
                value = int.from_bytes(data, byteorder='big', signed=True)
                if value == 1:
                    pydirectinput.mouseDown()
                    print("abaixou")
                elif value == 0:
                    pydirectinput.mouseUp()
                    print("levantou")
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