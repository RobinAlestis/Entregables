#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2
from beginner_tutorials.srv import MoveRobot, MoveRobotResponse  # Esta línea importa el servicio MoveRobot y MoveRobotResponse




class RobotController:

# Iniciacion de la clase de llamada:

    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True) # Inicia el nodo robot_controller
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Crea un publicador en el topico /cmd_vel, tipo twist. Parametro "queue_size" define cuántos mensajes pueden encolarse.
        rospy.Subscriber('/odom', Odometry, self.odom_callback) # Crea un suscriptor en el topico /odom, espera mensae tipo Odometry para llamar al metodo "self.odom_callback
        self.rate = rospy.Rate(10) # 10 Hz # Frecuencia de bucle de control.
        self.odom_position_x = 0.0 # Inicializacion de la posicion x de la odometeria del robot.
        self.odom_position_y = 0.0 # Inicializacion de la posicion y de la odometeria del robot.
        self.target_x = 1.0  # Posición objetivo en x
        self.target_y = 0.0  # Posición objetivo en y
        self.k_linear = 0.1  # Control lineal
        self.k_angular = 0.0  # Control angular         
        self.started = False  # Variable para verificar si el control ha comenzado

# Actualizacion de posiciones:    
    def odom_callback(self, odom_msg):
        self.odom_position_x = odom_msg.pose.pose.position.x # Actualizacion posicion x de la odometria.
        self.odom_position_y = odom_msg.pose.pose.position.y # Actualizacion posicion y de la odometria.
        
# Calcula y envia los comandos de velocidad del robot:  
    def move_to_target(self): # Calcula y envia los comandos de velocidad del robot
        dx = self.target_x - self.odom_position_x # Calcula la diferencia entre las coordenadas x actual y obetivo
        dy = self.target_y - self.odom_position_y # Calcula la diferencia entre las coordenadas y actual y obetivo
        distance_to_target = (dx**2 + dy**2)**0.5 # Calcula de la hipotenusa (triangulo en linea recta al obetivo)

        # Calcular el ángulo hacia el objetivo 
        target_angle = atan2(dy, dx)

        # Calcular el error de dirección
        error_angle = target_angle

        # Control angular para orientarse hacia el objetivo
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = self.k_angular * error_angle

        # Control lineal para avanzar hacia el objetivo (con velocidad proporcional a la distancia)
        if distance_to_target <= 1.0:  # Detener si está a menos de 1m. 
            cmd_vel_msg.linear.x = 0.0
        else:
            cmd_vel_msg.linear.x = min(self.k_linear * distance_to_target, 0.1)  # Limitar la velocidad lineal
        
        self.cmd_vel_pub.publish(cmd_vel_msg) 

# Actúa como un controlador de servicio:

    def move_robot_callback(self, req): # Funcion llamada
        self.started = True # Indica que el movimiento del robot esta activo, debe continuar con su movimiento
        while not rospy.is_shutdown(): #Bucle
            if self.started: # si esta activo, llama al metodo "move_to_target"
                self.move_to_target()
            else:
                self.stop() # de lo contrario, llama a Stop
            self.rate.sleep() #Controla la frecuencia de ejecucion del bucle.
        return MoveRobotResponse(True) #Devolucion de mensaje
        
#  Proporciona la parada del movimiento del robot:
    def stop(self):
        cmd_vel_msg = Twist() #Crea mensaje tipo twist, representa velocidad (lineal/angular)
        self.cmd_vel_pub.publish(cmd_vel_msg)  #Publicacion el mensaje
        
# Mantiene el nodo en ejecución y permitir que continúe recibiendo y respondiendo a mensajes:
    def control_loop(self):
        rospy.spin()
        
# Verifica si el script se esta ejecutando como programa principal:
if __name__ == '__main__':
    try:
        controller = RobotController() # Verifica si el script se esta eecutando como programa principal
        rospy.Service('move_robot', MoveRobot, controller.move_robot_callback) # Define un sevicio
        rospy.loginfo("Servicio de movimiento del robot iniciado.") # Confirmacion sevicio
        controller.control_loop() #Llama la metodo, responsable de controlar el movimiento del robot
    except rospy.ROSInterruptException:
        pass # Finaliza el programa cuando recibe una señal de interrupcion.



 
