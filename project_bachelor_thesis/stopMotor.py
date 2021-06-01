from matrix_lite import gpio


def motorInit():
    gpio.setMode(8, 'output')
    gpio.setFunction(8, 'pwm')
    gpio.setMode(14, 'output')
    gpio.setFunction(14, 'pwm')
    
    gpio.setFunction(12, 'DIGITAL')
    gpio.setFunction(10, 'DIGITAL')
    gpio.setFunction(6, 'DIGITAL')
    gpio.setFunction(4, 'DIGITAL')
    gpio.setMode(10, 'output')
    gpio.setMode(12, 'output')
    gpio.setMode(6, 'output')
    gpio.setMode(4, 'output')

def motorSpeedLeft(direction, speed):
    if direction == 1: ## FORWARD!
        gpio.setDigital(6, 'ON') #Drive forward: ON
        gpio.setDigital(4, 'OFF') #Drive backwards: OFF
    
    elif direction == 0: #BACKWARDS!!
        gpio.setDigital(6, 'OFF') #Drive forward: OFF
        gpio.setDigital(4, 'ON') #Drive backwards: ON

    if speed < 0:
        speed = 0
    if speed > 100:
        speed = 100
        
    gpio.setPWM({
        "pin": 8,
        "percentage": speed,
        "frequency": 1000,
        })

def motorSpeedRight(direction, speed):

    if direction == 1: ## FORWARD!
        gpio.setDigital(12, 'ON') #Drive forward: ON
        gpio.setDigital(10, 'OFF') #Drive backwards: OFF

    elif direction == 0: #BACKWARDS!!
        gpio.setDigital(12, 'OFF') #Drive forward: OFF
        gpio.setDigital(10, 'ON') #Drive backwards: ON

    if speed < 0:
        speed = 0
    if speed > 100:
        speed = 100
        
    gpio.setPWM({
        "pin": 14,
        "percentage": speed,
        "frequency": 1000,
        })
        
motorSpeedLeft(1, 0)
motorSpeedRight(1, 0)
        
