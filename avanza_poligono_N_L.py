#!/usr/bin/env python

import rospy
import actionlib
from mi_odometria_turtlebot.msg import ResponselienalorotationactionAction
from mi_odometria_turtlebot.msg import ResponselienalorotationactionFeedback, ResponselienalorotationactionResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

#Definicion de clase:
class RobotController:
# Iniciacion de la clase de llamada: 
    def __init__(self):
        rospy.init_node("robot_controller") # Iniciacion el nodo.
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) # Creacion del publicador.
        rospy.Subscriber("/odom", Odometry, self.odom_callback) # Creacion un suscriptor.
        
         # Definicion de variables.     
        self.rate = rospy.Rate(10)
        self.odom_position_x = 0.0
        self.odom_position_y = 0.0
        self.odom_orientation_yaw = 0.0
        self.inicial_position_x = 0.0
        self.inicial_position_y = 0.0
        self.target_z =0.0
        self.k_linear = 0.1
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
        
# Se encarga de asignar valores a las variables 'inicial_position_x' e 'inicial_position_y' basadas en los valores actuales de 'odom_position_x' y 'odom_position_y', respectivamente:
    def target_Distancia(self):
        self.inicial_position_x = self.odom_position_x  
        self.inicial_position_y = self.odom_position_y

# Define el movimiento del robot segun tipo de movimiento:     
    def execute_movement(self, goal):
        rospy.loginfo("Movimiento recibido") # Registra un mensaje de información indicando que se ha recibido un movimiento.
        result = ResponselienalorotationactionResult()
        self.started = True # Inicializa la variable 'started' como 'True'
        sides = goal.NumSides # Obtiene el número de lados del polígono objetivo del objeto 'goal'.
        side_length =  goal.Distancia # Obtiene la distancia del lado del polígono objetivo del objeto 'goal'.
        angle = 360.0 / sides # Calcula el ángulo necesario para cada giro basado en el número de lados del polígono objetivo.
        rospy.loginfo(side_length) # Registra la longitud del lado del polígono objetivo.
        rospy.loginfo(angle) # Registra el ángulo calculado para cada giro
        
        for _ in range(sides): # Inicio bucle:
            self.started=True 
            self.target_Distancia() # Llama al método 'target_Distancia' para establecer la posición inicial.
            
            while self.started: # Inicia un bucle que ejecutará el movimiento hasta que la variable 'started' sea falsa
                self.move_to_target(side_length) # Llama al método 'move_to_target' para mover el robot hacia el objetivo con la distancia del lado especificada.
                rospy.sleep(1)
            self.stop() # Detiene el movimiento del robot.
            rospy.sleep(1)
            self.started=True
            self.target_Angle(angle) #  Llama al método 'target_Angle' para establecer el ángulo objetivo.
            while self.started:
                self.rotate_to_target()
                rospy.sleep(1)
            self.stop() # Detiene el movimiento de rotación del robot.
            rospy.sleep(1)        
       
        self.stop()
        self.rate.sleep()

        result.MoveRobot = True # Indica que el movimiento del robot se ha completado con éxito.
        self.server.set_succeeded(result)
   
# Calcula el ángulo objetivo al que el robot debe rotar:
    def target_Angle(self,angle):
    
    # Calcular el ángulo de rotación:
        rotation_angle = math.radians(angle)       
        target_angle = self.odom_orientation_yaw + rotation_angle # Calcula el ángulo objetivo sumando el ángulo actual del robot ('self.odom_orientation_yaw') y el ángulo de rotación deseado.
        
        if target_angle >= math.radians(180): #  Verifica si el ángulo objetivo calculado es mayor o igual a 180 grados (en radianes).
          target_angle-=math.radians(360) #  Si el ángulo calculado excede 180 grados, resta 360 grados (en radianes) para obtener un ángulo equivalente pero en el rango [-π, π].
        self.target_z = target_angle 

  # Calcula los comandos de velocidad que se enviarán al robot para moverlo hacia un objetivo específico        
    def move_to_target(self, target_distance):
        feedback = ResponselienalorotationactionFeedback()
        dx = self.odom_position_x - self.inicial_position_x
        dy = self.odom_position_y - self.inicial_position_y
        distance_to_target = math.sqrt(dx ** 2 + dy ** 2) #   Calcula la distancia euclidiana entre la posición actual del robot y el objetivo
        cmd_vel_msg = Twist() # Iniciacion mensae a robot, envia comandos de velocidad.

        if distance_to_target >= target_distance: # Verifica si la distancia al objetivo es mayor o igual que la distancia objetivo.
            cmd_vel_msg.linear.x = 0.0
            self.started = False
        else:
            cmd_vel_msg.linear.x = 0.1 # Si la distancia al objetivo es menor que la distancia objetivo, establece una velocidad lineal baja de 0.1 para que el robot se mueva hacia adelante hacia el objetivo.

        feedback.Diferencia = distance_to_target
        self.server.publish_feedback(feedback) # Publica la retroalimentación calculada en el servidor para que pueda ser utilizada por el cliente que solicitó el movimiento.
        self.cmd_vel_pub.publish(cmd_vel_msg) #  Publica el mensaje de velocidad calculado para que el robot pueda moverse según las instrucciones.

# Esta función,gira el robot hacia un ángulo objetivo específico. Aquí está el desglose línea por línea:
    def rotate_to_target(self):
        feedback = ResponselienalorotationactionFeedback()
        cmd_vel_msg = Twist() # Inicializa un mensaje de tipo 'Twist()', que se utiliza para enviar comandos de velocidad de rotación al robot.
        dz = self.target_z - self.odom_orientation_yaw # Calcula la diferencia entre el ángulo objetivo ('self.target_z') y la orientación actual del robot ('self.odom_orientation_yaw').

        if abs(dz) <= math.radians(10): #  Verifica si la diferencia de ángulo es menor o igual a 10 grados en radianes.
            cmd_vel_msg.angular.z = 0.0 # Si la diferencia de ángulo es menor o igual a 10 grados, establece la velocidad angular del robot en cero, deteniendo la rotación.
            self.started = False
        else:
            cmd_vel_msg.angular.z = 0.3 # Si la diferencia de ángulo es mayor que 10 grados, establece una velocidad angular de rotación de 0.3 para que el robot gire hacia el ángulo objetivo

        feedback.Diferencia = dz
        self.server.publish_feedback(feedback) # Publica la retroalimentación calculada en el servidor para que pueda ser utilizada por el cliente que solicitó la rotación.

        self.cmd_vel_pub.publish(cmd_vel_msg) # Publica el mensaje de velocidad angular calculado para que el robot pueda rotar según las instrucciones.

# Detiene el movimiento del robot, asegurándose de que tanto su velocidad lineal como angular sean cero:
    def stop(self):
        cmd_vel_msg = Twist() #  Inicializa un mensaje de tipo 'Twist()', que se utiliza para enviar comandos de velocidad al robot.
        cmd_vel_msg.linear.x = 0.0 # Establece la velocidad lineal del robot en cero, lo que significa que el robot no se moverá hacia adelante o hacia atrás
        cmd_vel_msg.angular.z = 0.0 # Establece la velocidad angular del robot en cero, lo que significa que el robot no girará.
        self.cmd_vel_pub.publish(cmd_vel_msg) # Publica el mensaje de velocidad calculado para que el robot detenga cualquier movimiento en curso y se quede en su posición actual.

if __name__ == "__main__":
    try:
        controller = RobotController()
        rospy.loginfo("Servicio de movimiento del robot iniciado.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

