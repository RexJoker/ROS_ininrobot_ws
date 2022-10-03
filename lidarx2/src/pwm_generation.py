import RPi.GPIO as GPIO

ledpin = 18				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BCM)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,10000)		#create PWM instance with frequency
print("PWM Generation starts") 
pi_pwm.start(100)				#start PWM of required Duty Cycle

input()

pi_pwm.stop()
