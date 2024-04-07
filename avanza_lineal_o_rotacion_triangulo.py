#!/usr/bin/env python

import rospy
import actionlib
from mi_odometria_turtlebot.msg import ResponselienalorotationactionAction
from mi_odometria_turtlebot.msg import ResponselienalorotationactionFeedback, ResponselienalorotationactionResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

# Valor pequeño para detener el robot gradualmente
STOP_DISTANCE = 0.1

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
        self.odom_orientation_roll = 0.0
        self.odom_orientation_pitch = 0.0
        self.target_x = 1.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.k_linear = 0.1
        self.k_angular = 0.5
        self.started = False # Indica que el controlador del robot no ha comenzado a funcionar.
        self.posicion_inicial_x= 0.0
        self.posicion_inicial_y= 0.0
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
        
 # Define el movimiento del robot segun tipo de movimiento:   
    def execute_movement(self, goal): 
        rospy.loginfo("Movimiento recibido")
        result = ResponselienalorotationactionResult()
        self.started = True
       
       # Calcular el ángulo de rotación para formar un triángulo equilátero
        rotation_angle = math.radians(goal.Distancia)  # 120 grados para un triángulo equilátero       
        self.posicion_inicial_x= self.odom_position_x
        self.posicion_inicial_y= self.odom_position_y
        target_angle = self.odom_orientation_yaw + rotation_angle
        
        if target_angle >= math.radians(180):
          target_angle-=math.radians(360)
        self.target_z = target_angle 
        rospy.loginfo(target_angle)
        while self.started:  
          
          # Realizar movimiento lineal si el objetivo es 1
          
          if goal.Rotacion0 == 1:

              self.move_to_target(goal.Distancia) 

              rospy.sleep(1)
       
          # Realizar rotación si el objetivo es 0
          elif goal.Rotacion0 == 0:           
             
    
              # Establecer el ángulo objetivo de rotación
             
             rospy.loginfo(self.odom_orientation_yaw) 
             rospy.loginfo(self.target_z)      
             self.rotate_to_target()          
             rospy.sleep(1) 
             #self.rotate_to_target(target_angle) 
          self.stop()
          self.rate.sleep()  
        
        # Establecer el resultado de la acción como completado
        result.MoveRobot=True
        self.server.set_succeeded(result)
                

   # Calcula los comandos de velocidad que se enviarán al robot para moverlo hacia un objetivo específico
    def move_to_target(self, obetivo):
        feedback = ResponselienalorotationactionFeedback()
        dx = self.odom_position_x - self.posicion_inicial_x
        dy = self.odom_position_y - self.posicion_inicial_y
        distance_to_target = math.sqrt(dx**2 + dy**2) #   Calcula la distancia euclidiana entre la posición actual del robot y el objetivo
        cmd_vel_msg = Twist() # Iniciacion mensae a robot, envia comandos de velocidad.
        
        if distance_to_target >= obetivo:
            cmd_vel_msg.linear.x = 0.0
            self.started = False  # cambiarlo en el otro recordar
            self.stop()
        else:
            
            cmd_vel_msg.linear.x = (0.08)
        feedback.Diferencia=distance_to_target
        self.server.publish_feedback(feedback)
        self.cmd_vel_pub.publish(cmd_vel_msg)

# Calcula y envía comandos de velocidad al robot para que gire hacia un ángulo objetivo.
    def rotate_to_target(self):
        feedback = ResponselienalorotationactionFeedback()
        # Calcula la diferencia entre el ángulo actual y el ángulo objetivo
        dz = self.target_z - self.odom_orientation_yaw 
        # Crea el mensage de velocidad para el robot
        cmd_vel_msg = Twist()
        # Si la diferencia de ángulo ya es pequeña, detiene el movimiento
        # if dz <= 0.1:
        if abs(dz) <= math.radians(10): 
            cmd_vel_msg.angular.z = 0.0
            self.started = False
            self.stop()
        # De lo contrario, el robot sigue rotando con control proporcional
        else:
            cmd_vel_msg.angular.z = (0.3)
        feedback.Diferencia=dz
        self.server.publish_feedback(feedback) 
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        # Envía un mensaje de velocidad vacío al robot, para que el robot se detenga. 
    def stop(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)

       # Crea y ejecuta el nodo que contiene el controlador del robot, y maneja cualquier interrupción de manera adecuada para asegurar una terminación limpia del nodo.
if __name__ == "__main__":
   try:
        controller = RobotController()
        rospy.loginfo("Servicio de movimiento del robot iniciado.")
        rospy.spin()
   except rospy.ROSInterruptException:
        pass
