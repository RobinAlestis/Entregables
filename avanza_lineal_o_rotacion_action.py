#!/usr/bin/env python

import rospy
import actionlib
from mi_odometria_turtlebot.msg import ResponselienalorotationactionAction
from mi_odometria_turtlebot.msg import ResponselienalorotationactionFeedback, ResponselienalorotationactionResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion


# Definicion de clase:

class RobotController: 

# Iniciacion de la clase de llamada:  
    def __init__(self):
        rospy.init_node("robot_controller") # Iniciacion el nodo.
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) # Creacion del publicador.
        rospy.Subscriber("/odom", Odometry, self.odom_callback)  # Creacion un suscriptor.
        
        # Definicion de variables.
        self.rate = rospy.Rate(10) 
        self.odom_position_x = 0.0
        self.odom_position_y = 0.0
        self.odom_orientation_yaw = 0.0
        self.odom_orientation_roll = 0.0
        self.odom_orientation_pitch = 0.0
        self.target_x = 1.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.k_linear = 0.1
        self.k_angular = 0.5
        self.started = False # Indica que el controlador del robot no ha comenzado a funcionar.
        self.server = actionlib.SimpleActionServer(
            "move_robot", ResponselienalorotationactionAction, self.execute_movement, False # Crea un servidor de accion simple.
        )
        self.server.start() # Con esta llamada, el servidor comenzará a escuchar las solicitudes de acción y a ejecutar.

 #Definicion del metodo.(actualiza la posicion actual del Robot con coordenadas y su orientacion):
    
    def odom_callback(self, odom_msg): 
        
        # Obtiene la posición actual en el eje xy
        self.odom_position_x = odom_msg.pose.pose.position.x
        self.odom_position_y = odom_msg.pose.pose.position.y
        
        # Obtiene la orientación actual en cuaternios, la pasa a ángulos de euler       
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (
            self.odom_orientation_roll,
            self.odom_orientation_pitch,
            self.odom_orientation_yaw,
        ) = euler_from_quaternion(orientation_list)
        
#Se encarga de manejar la acción recibida por el servidor de acción:

    def execute_movement(self, goal):
        rospy.loginfo("Movimiento recibido") # Mensaje de registro para indicar que se ha recibido una solicitud de movimiento

        result = ResponselienalorotationactionResult() # Crea un objeto de tipo ResponselienalorotationactionResult()
        self.started = True # Inicializa la variable 
        
        while not rospy.is_shutdown(): # Inicialzacion del Bucle.
            
            if self.started: # Comprueba si se ha inicializado el movimiento.
                if goal.Rotacion0 == 1: # Se ha solicitado un movimiento lineal.
                    self.target_x = goal.Distancia # Indica la distancia que debe de moverse.
                    self.move_to_target() # Calcular y enviar los comandos de velocidad necesarios al robot para que se mueva hacia el objetivo.
                elif goal.Rotacion0 == 0: # Se ha solicitado un movimiento de rotacion.
                    # Transforma la entrada en grados sexagesimales a radianes
                    self.target_z = goal.Distancia * (math.pi)/180
                    # Inicia la rotación
                    self.rotate_to_target()
                    
            else: # Detiene el movimiento, y sale del bucle.
                self.stop()
                break  # CAMBIAR EN EL OTRO
                
            self.rate.sleep()
        result.MoveRobot=True # Establece el resultado de la accion y avisa al cliente que la accion ha sido completada.
        self.server.set_succeeded(result)
                

# Calcula los comandos de velocidad que se enviarán al robot para moverlo hacia un objetivo específico
    
    def move_to_target(self):
        feedback = ResponselienalorotationactionFeedback() #  Crea un objeto de tipo ResponselienalorotationactionFeedback(), se utiliza para retroalimentar al cliente sobre el progreso del movimiento.
        
        # calcula la diferencia en las coordenadas x e y entre la posición actual del robot y las coordenadas x e y del objetivo.        
        dx = self.target_x - self.odom_position_x
        dy = self.target_y - self.odom_position_y  
        
        # Calcula la distancia euclidiana entre la posición actual del robot y el objetivo
        distance_to_target = (dx**2 + dy**2) ** 0.5
        cmd_vel_msg = Twist() # Iniciacion mensaje a robot, envia comandos de velocidad.
        
        # Verifica si la distancia al objetivo es menor o igual a 1.0. Si es así, significa que el robot está lo suficientemente cerca del objetivo y debe detenerse.
        if distance_to_target <= 1.0:
            cmd_vel_msg.linear.x = 0.0
            self.started = False  # CAMBIAR EN EL OTRO
            
        # Si la distancia al objetivo es mayor que 1.0, se calcula la velocidad lineal para acercarse al objetivo. La velocidad lineal se ajusta proporcionalmente a la distancia al objetivo
        else:
            cmd_vel_msg.linear.x = min(self.k_linear * distance_to_target, 0.1)
        
        feedback.Diferencia=distance_to_target # Se actualiza el campo Diferencia del objeto feedback con la distancia al objetivo.
        self.server.publish_feedback(feedback) # Se publica la retroalimentación, permite al cliente recibir actualizaciones sobre el progreso del movimiento
        self.cmd_vel_pub.publish(cmd_vel_msg) # Se publica el mensaje de velocidad, lo que envía los comandos de velocidad al robot para que se mueva según lo calculado.

# Calcula y envía comandos de velocidad al robot para que gire hacia un ángulo objetivo.
    
    def rotate_to_target(self):
        feedback = ResponselienalorotationactionFeedback() 
        # Calcula la diferencia entre el ángulo actual y el ángulo objetivo
        dz = self.target_z - self.odom_orientation_yaw 
        # Crea el mensage de velocidad para el robot
        cmd_vel_msg = Twist()
        # Si la diferencia de ángulo ya es pequeña, detiene el movimiento
        if dz <= 0.1:
            cmd_vel_msg.angular.z = 0.0
            self.started = False
        # De lo contrario, el robot sigue rotando con control proporcional
        else:
            cmd_vel_msg.angular.z = min(self.k_angular * dz, 0.5)
        feedback.Diferencia=dz
        self.server.publish_feedback(feedback) 
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        # Envía un mensaje de velocidad vacío al robot, para que el robot se detenga. 
    def stop(self):
        cmd_vel_msg = Twist()
        self.cmd_vel_pub.publish(cmd_vel_msg)

# Crea y ejecuta el nodo que contiene el controlador del robot, y maneja cualquier interrupción de manera adecuada para asegurar una terminación limpia del nodo.
if __name__ == "__main__":
   try:
        controller = RobotController()
        rospy.loginfo("Servicio de movimiento del robot iniciado.")
        rospy.spin()
   except rospy.ROSInterruptException:
        pass
