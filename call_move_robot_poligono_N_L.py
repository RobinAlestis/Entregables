#!/usr/bin/env python

import rospy
import actionlib
from mi_odometria_turtlebot.msg import ResponselienalorotationactionAction, ResponselienalorotationactionGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion


# Se encarga de enviar una solicitud de movimiento al servidor de acción y esperar hasta que el movimiento sea completado antes de continuar con la ejecución del programa:
def move_robot_client(move_type, value): 
    rospy.init_node('call_move_robot_poligono_N_L') # Inicia el nodo
    client = actionlib.SimpleActionClient('move_robot', ResponselienalorotationactionAction) #Crea un cliente
    client.wait_for_server()  # Espera a que el servidor de acción esté disponible antes de continuar.


    goal = ResponselienalorotationactionGoal() #  Crea un objeto, que es el mensaje de objetivo definido para la acción
    goal.NumSides = move_type # Asigna el valor de numeros de lados elegido para el poligono.
    goal.Distancia = value # Convertir el valor de distancia a un entero antes de asignarlo al objetivo

    rospy.loginfo("Enviado solicitud de movimiento...") # Registra un mensaje informativo
    client.send_goal(goal, feedback_cb=feedback_callback)

    client.wait_for_result() # Espera hasta que el resultado de la acción esté disponible
    rospy.loginfo("Movimiento completado")

#Devolucion de llamada, que se utiliza para manejar la información de retroalimentación:  
def feedback_callback(feedback):
    rospy.loginfo(feedback.Diferencia) 

# Define la generacion de un poligono con un numero especifico de lados y longitud especifica:
def generate_polygon(sides, side_length): # : 'sides', que indica el número de lados del polígono, y 'side_length', que indica la longitud de cada lado del polígono.
   
    move_robot_client(sides, side_length) #  Moverá el robot para generar el polígono deseado con los parámetros especificados.
    rospy.sleep(2)
    rospy.loginfo("Polígono de {} lados completado.".format(sides)) # Registra un mensaje informativo utilizando 'rospy.loginfo', indicando que se ha completado la generación del polígono con el número de lados especificado.

# Se ejecuta cuando el script es ejecutado directamente: 
if __name__ == '__main__': # verifica que el script esta siendo ejecutado.
    try:
        sides_str = input("Ingrese el número de lados del polígono: ") # Solicita al usuario que ingrese el número de lados del polígono.
        side_length_str = input("Ingrese la longitud de los lados del polígono: ") # Solicita al usuario que ingrese la longitud de los lados del polígono.

        try:
            sides = int(sides_str) # Convierte la entrada de usuario 'sides_str' a un entero, que representa el número de lados del polígono.
            side_length = int(side_length_str) # Convierte la entrada de usuario 'side_length_str' a un entero, que representa la longitud de los lados del polígono.
            generate_polygon(sides, side_length) #  Llama a la función 'generate_polygon' con los valores proporcionados por el usuario para 'sides' y 'side_length'.
        except ValueError: #  Captura una excepción específica, en este caso, cuando la conversión de cadena a entero falla, lo que significa que el usuario no ingresó un número válido.
            rospy.logwarn("Por favor, ingrese números válidos.")

    except rospy.ROSInterruptException:
        pass

