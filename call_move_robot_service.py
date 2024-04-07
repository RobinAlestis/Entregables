
#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import MoveRobot, MoveRobotResponse  # Esta línea importa el servicio MoveRobot y MoveRobotResponse

# Espera a que el servicio 'move_robot' esté disponible, luego crea un cliente para este servicio y lo llama: 
def move_robot_client():
    rospy.wait_for_service('move_robot')  # Espera a que el servicio esté disponible
    try:
        move_robot = rospy.ServiceProxy('move_robot', MoveRobot)  # Crea un cliente del servicio
        move_robot()  # Llama al servicio
        rospy.loginfo("¡El robot está en movimiento!")  # Mensaje de información
    except rospy.ServiceException as e:
        rospy.logerr("Error al llamar al servicio: %s" % e)  # Mensaje de error

# Determina si el script se está ejecutando como un programa independiente o si se está importando como un módulo en otro script:
if __name__ == '__main__':
    rospy.init_node('move_robot_client')  # Inicializa el nodo del cliente
    move_robot_client()  # Llama a la función del cliente

