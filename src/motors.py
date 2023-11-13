#!/usr/bin/env python3

import time
import pigpio

class ESC:
    MIN_WIDTH = 1000
    MAX_WIDTH = 2400

    def __init__(self, pin1:int, pin2:int = None)-> None:
        self.conn = pigpio.pi()
        self.pin1 = pin1
        self.pin2 = pin2
        
        self.conn.set_mode(self.pin1, pigpio.OUTPUT)  #pigpio.PWM(12)  
        
        self.conn.set_PWM_frequency(self.pin1, 50)
        
        if self.pin2 != None:
            self.conn.set_mode(self.pin2, pigpio.OUTPUT)
            self.conn.set_PWM_frequency(self.pin2, 50)


    def pwm(self, width: int)-> None:
        self.conn.set_servo_pulsewidth(self.pin1, width)
        if self.pin2 != None:
            self.conn.set_servo_pulsewidth(self.pin2, width)
        

    def calibrate(self)-> None:
        print('"menor" velocidade')
        self.pwm(self.MIN_WIDTH)
        time.sleep(2)
        
        print('"maior" velocidade')
        #self.pwm(self.MAX_WIDTH)
        #time.sleep(5)
        
        print("calibrado pai")
        self.pwm(self.MIN_WIDTH)
        
    def arm(self)-> None:
        print("armando")
        self.pwm(self.MIN_WIDTH)
        time.sleep(2)
        
        print("armado")
        
    def halt(self)-> None:
        print("parando")
        self.pwm(self.MIN_WIDTH)
        
        print("ta safe")
        self.pwm(0)
        
        print("desligando GPIO.")
        self.conn.stop()
        
        print("já era")
    
    def manual_control(self)-> None:
        a = input()
        veloc = self.conn.get_servo_pulsewidth(self.pin1)
        
        while a != 'x':
            if a == 'q':
                veloc += 10
                self.pwm(veloc)
            
            if a == 'a':
                veloc -= 10
                self.pwm(veloc)
            
            if a == 'e':
                veloc += 100
                self.pwm(veloc)
            
            if a == 'd':
                veloc -= 100
                self.pwm(veloc)
            
            print(veloc)
            a = input()
            
    def control(self, width:int)-> None:
        if width < self.MAX_WIDTH and width > self.MIN_WIDTH:
            self.pwm(width)
    
    def test(self) ->None:
        self.pwm(self.MIN_WIDTH)
        
        step = 100
        print("acelerando")
        for veloc in range(self.MIN_WIDTH, self.MAX_WIDTH, step):
            self.pwm(veloc)
            print(veloc)
            time.sleep(1)
        
        time.sleep(2)  
        print("parando")
        for veloc in range(self.MAX_WIDTH, self.MIN_WIDTH, -step):
            print(veloc)
            self.pwm(veloc)
            time.sleep(.5)  

class Servo:
    MIN_WIDTH = 600 #menor angulo em teoria
    MAX_WIDTH = 2400 #maior angulo em teoria, cuidado ppra n quebrar essa porra
     
    def __init__(self, pin:int)-> None:
        self.pin = pin
        self.conn = pigpio.pi()
        
    def control(self) -> None:
        self.conn.set_servo_pulsewidth(self.pin ,1750) # máximo à direita
        time.sleep(1.5)
        
        self.conn.set_servo_pulsewidth(self.pin ,1250) # centro
        time.sleep(1.5)
        
        self.conn.set_servo_pulsewidth(self.pin ,750) # máximo á esquerda
        time.sleep(1.5)
        
    def manual_control(self)-> None:
        width = int(input('qual o "angulo":'))
        
        #tem q converter pra graus, mas fdc
        while width != "x":
            while width < self.MIN_WIDTH or width > self.MAX_WIDTH:
                width = input("ta fazendo merda: ")    
                
            self.conn.set_servo_pulsewidth(self.pin, width)
            width = int(input('angulo novo: '))
#NOTA: as escs estão sendo controlada separadamente, dps tem q ver isso

if __name__ == "__main__":

    servo = Servo(pin=18)
    
    esc1 = ESC(pin1=13)
    esc2 = ESC(pin1=12)

    while True:
        inp = input("vai mexer onde?")
    
        if inp == "esc":
            
            esc1.calibrate()
            esc2.calibrate()
            
            esc1.pwm(1070)
            time.sleep(2)
            esc1.pwm(1250)
            time.sleep(2)
            esc1.pwm(1500)
            time.sleep(2)
            esc1.pwm(1600)

            resp = input("inflou o saco?")
            if resp == "s": 
                esc2.arm() #deixa a velocidade dele fixa 
                esc2.pwm(1040)
                esc2.pwm(1400)
                time.sleep(2)
                esc2.pwm(1600)
                time.sleep(2)
                esc2.pwm(1800)
        

            elif resp == "x":
                esc1.halt()
                esc2.halt()
                break
    
            #esc1.test()
            #esc2.test()
        elif inp == "servo":
            servo.control()
           # sOUn = input("quer brincar?")
         #   if sOUn == 's':
          #      servo.manual_control()
           # elif sOUn == 'x':
            #    esc1.halt()
             #   esc2.halt()
              #  break
            
        elif inp == "x":
            esc1.halt()
            esc2.halt()
            break
