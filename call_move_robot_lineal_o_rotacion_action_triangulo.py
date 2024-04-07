  #!/usr/bin/env python

import rospy
import actionlib
from mi_odometria_turtlebot.msg import ResponselienalorotationactionAction,ResponselienalorotationactionGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

# Valor pequeño para detener el robot gradualmente
STOP_DISTANCE = 0.1

# Se encarga de enviar una solicitud de movimiento al servidor de acción y esperar hasta que el movimiento sea completado antes de continuar con la ejecución del programa:
def move_robot_client(move_type, value):
    rospy.init_node('call_move_robot_lineal_o_rotacion_action_triangulo') # Inicia el nodo
    client = actionlib.SimpleActionClient('move_robot', ResponselienalorotationactionAction) #Crea un cliente
    client.wait_for_server() # Espera a que el servidor de acción esté disponible antes de continuar.

    goal = ResponselienalorotationactionGoal() #  Crea un objeto, que es el mensaje de objetivo definido para la acción
    goal.Rotacion0 = move_type  # 1 para lineal, 0 para rotacion. 
    goal.Distancia = int(value)  # Convertir el valor de distancia a un entero antes de asignarlo al objetivo

    rospy.loginfo("Enviado solicitud de movimiento...") # Registra un mensaje informativo
    client.send_goal(goal,feedback_cb=feedback_callback ) 

    client.wait_for_result() # Espera hasta que el resultado de la acción esté disponible
    rospy.loginfo("Movimiento completado") 

#Devolucion de llamada, que se utiliza para manejar la información de retroalimentación:    
def feedback_callback(feedback):
    rospy.loginfo(feedback.Diferencia)

if __name__ == '__main__':
    try:    
        
        while True:
            L_str = input("Ingrese la longitud del lado del triángulo: ")
            try:
                L = int(float(L_str))  # Convertir a flotante y luego a entero
                break
            except ValueError:
                rospy.logwarn("Por favor, ingrese un número válido.")

       # Realizar tres movimientos de línea y tres de rotación para formar el triángulo equilátero:
       
        for _ in range(3):
            move_robot_client(move_type=1, value=L) # Mover hacia adelante
            rospy.sleep(2)  # Esperar a que el movimiento lineal se complete antes de rotar
            move_robot_client(move_type=0, value=120) # Rotar 120 grados para formar el primer lado del triángulo 
            rospy.sleep(2)  # Esperar a que el movimiento lineal se complete antes de rotar 

        
        # Detener gradualmente el robot después de completar el triángulo
        move_robot_client(move_type=1, value=STOP_DISTANCE)  # Mover una pequeña distancia para detener gradualmente
        rospy.sleep(2)  # Esperar un poco antes de detener completamente el robot   
        rospy.loginfo("Triángulo completado.")

    except rospy.ROSInterruptException:
        pass

