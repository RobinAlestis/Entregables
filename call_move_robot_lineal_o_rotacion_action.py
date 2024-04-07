#!/usr/bin/env python

import rospy
import actionlib
from mi_odometria_turtlebot.msg import ResponselienalorotationactionAction,ResponselienalorotationactionGoal

# Se encarga de enviar una solicitud de movimiento al servidor de acción y esperar hasta que el movimiento sea completado antes de continuar con la ejecución del programa:
def move_robot_client(move_type, distancia):
    
    rospy.init_node('call_move_robot_lineal_o_rotacion_action') # Inicia el nodo
    client = actionlib.SimpleActionClient('move_robot', ResponselienalorotationactionAction) #Crea un cliente
    client.wait_for_server() # Espera a que el servidor de acción esté disponible antes de continuar.

    goal = ResponselienalorotationactionGoal() #  Crea un objeto, que es el mensaje de objetivo definido para la acción
   # Dependiendo del tipo de movimiento ,si move_type es 'linear', se establece en 1; de lo contrario, se establece en 0.
    if move_type == 'linear':
        goal.Rotacion0 = 1 
    else:
        goal.Rotacion0 = 0
    goal.Distancia = distancia # 

    rospy.loginfo("Enviado solicitud de movimiento...") # Registra un mensaje informativo
    client.send_goal(goal,feedback_cb=feedback_callback ) 

    client.wait_for_result() # Espera hasta que el resultado de la acción esté disponible
    rospy.loginfo("Movimiento completado") 

#Devolucion de llamada, que se utiliza para manejar la información de retroalimentación. 

def feedback_callback(feedback):
    rospy.loginfo(feedback.Diferencia)

if __name__ == '__main__':
    try:
        move_robot_client(move_type='linear', distancia=1)
        move_robot_client(move_type='rotation', distancia=45)
    except rospy.ROSInterruptException:
        pass

