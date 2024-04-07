#!/usr/bin/env python
 
import rospy
from mi_odometria_turtlebot.srv import Responselienalorotation, ResponselienalorotationRequest
 
# Llama a un sevicio:
def move_robot_client(move_type,distancia):
    rospy.wait_for_service('move_robot') # Espera que este disponible el servicio.
    try:
        move_robot = rospy.ServiceProxy('move_robot', Responselienalorotation)
        response = ResponselienalorotationRequest() #Almacena los datos que se enviaran al servicio.
        if move_type=='linear': 
        	response.Rotacion0=1
        else:
        	response.Rotacion0=0
        response.Distancia=distancia	
        rospy.loginfo("¡El robot está en movimiento!")
        resultado=move_robot(response)
    
    except rospy.ServiceException as e:
        rospy.logerr("Error al llamar al servicio: %s" % e)

 
if __name__ == '__main__':
    rospy.init_node('move_robot_client')
   
   
    try:
        move_robot_client(move_type='linear', distancia=1)     # Llama a la función del cliente con los parámetros para movimiento lineal (tipo 'linear') o rotación (tipo 'rotation')
        move_robot_client(move_type='rotation',distancia=45) # Especifique los valores de movimiento o rotación según sea necesario
    except rospy.ROSInterruptException:
        pass


