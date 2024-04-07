#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys, select, termios, tty
from math import atan2

class RobotController:

# Iniciacion de la clase de llamada:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True) # Inicia el nodo robot_controller
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Crea un publicador en el topico /cmd_vel, tipo twist. Parametro "queue_size" define cuántos mensajes pueden encolarse.
        rospy.Subscriber('/odom', Odometry, self.odom_callback) # Crea un suscriptor en el topico /odom, espera mensae tipo Odometry para llamar al metodo "self.odom_callback"
        self.rate = rospy.Rate(10) # 10 Hz # Frecuencia de bucle de control.
        self.odom_position_x = 0.0 # Inicializacion de la posicion x de la odometeria del robot.
        self.odom_position_y = 0.0 # Inicializacion de la posicion y de la odometeria del robot.
        self.target_x = 1.0  # Posición objetivo en x
        self.target_y = 0.0  # Posición objetivo en y
        self.k_linear = 0.1  # Control lineal
        self.k_angular = 0.0 # Control angular         
        self.started = False  # Variable para verificar si el control ha comenzado

# Actualizacion de posiciones:
    def odom_callback(self, odom_msg):
        self.odom_position_x = odom_msg.pose.pose.position.x # Actualizacion posicion x de la odometria.
        self.odom_position_y = odom_msg.pose.pose.position.y # Actualizacion posicion y de la odometria.

# Calcula y envia los comandos de velocidad del robot:   
    def move_to_target(self): 
        dx = self.target_x - self.odom_position_x # Calcula la diferencia entre las coordenadas x actual y obetivo
        dy = self.target_y - self.odom_position_y # Calcula la diferencia entre las coordenadas y actual y obetivo
        distance_to_target = (dx**2 + dy**2) # Calcula de la hipotenusa (triangulo en linea recta al obetivo)

        # Calcular el ángulo hacia el objetivo
        target_angle = atan2(dy, dx)

        # Calcular el error de dirección
        error_angle = target_angle

        # Control angular para orientarse hacia el objetivo
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = self.k_angular * error_angle
        
        # Control lineal para avanzar hacia el objetivo (con velocidad proporcional a la distancia)
        if distance_to_target <= 1.0:  # Detener si está a menos de 1m del objetivo en el eje x
            cmd_vel_msg.linear.x = 0.0
        else:
            cmd_vel_msg.linear.x = min(self.k_linear * distance_to_target, 0.1)  # Limitar la velocidad lineal
        
        self.cmd_vel_pub.publish(cmd_vel_msg) #Publicacion el mensaje

#  Proporciona la parada del movimiento del robot:
    def stop(self):
        cmd_vel_msg = Twist() #Crea mensaje tipo twist, representa velocidad (lineal/angular)
        self.cmd_vel_pub.publish(cmd_vel_msg) #Publicacion el mensaje
    
    def control_loop(self):
        while not rospy.is_shutdown():
        
            if self.odom_position_x != 0.0 and self.odom_position_y != 0.0:  # Esperar a que la odometría esté disponible
                self.move_to_target() # Llama al metodo para que el robot avance.
            self.rate.sleep() # Asegura que el bucle se ejecute a una frecuencia constante.(10 Hz)

# Lee una tecla presionada por el usuario desde la entrada estándar:
def getKey(): 
    tty.setraw(sys.stdin.fileno()) #  lee una sola tecla a la vez sin necesidad de presionar "Enter".
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # Espera hasta 0.1sg para que la entrada esté disponible para lectura, si lo esta, el programa procede a leerla.
    if rlist:
        key = sys.stdin.read(1) #: Si hay datos disponibles, lee un solo carácter de la entrada estándar (sys.stdin) y lo almacena en la variable key
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) # Restaura la configuración original de la terminal después de leer la tecla
    return key

# Verifica si el script se esta eecutando como programa principal:
if __name__ == '__main__': 
    settings = termios.tcgetattr(sys.stdin) # Guarda la congiguracion actual de la terminal.
    try:
        controller = RobotController() # Define la variable 
        print("Presione cualquier tecla para iniciar el movimiento...")
        while True:
            key = getKey() #llama a la funcion key
            if key:
                controller.started = True # Indica que el movimiento del robot ha sido iniciado.
                break
        controller.control_loop() #Llama la metodo, responsable de controlar el movimiento del robot
    except rospy.ROSInterruptException:
        pass # Finaliza el programa cuando recibe una señal de interrupcion.


 
