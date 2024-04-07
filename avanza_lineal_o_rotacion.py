
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mi_odometria_turtlebot.srv import (
    Responselienalorotation,
    ResponselienalorotationResponse,
)
 
 # Definicion de clase.
class RobotController:

# Iniciacion de la clase de llamada:    
    def __init__(self):
        rospy.init_node("robot_controller", anonymous=True) # Iniciacion el nodo.
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) # Creacion del publicador.
        rospy.Subscriber("/odom", Odometry, self.odom_callback) # Creacion un suscriptor.
        self.rate = rospy.Rate(10)
        self.odom_position_x = 0.0
        self.odom_position_y = 0.0
        self.odom_orientation_roll = 0.0
        self.odom_orientation_pitch = 0.0
        self.odom_orientation_yaw = 0.0
        self.target_x = 1.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.k_linear = 0.1
        self.k_angular = 0.5
        self.started = False # Indica que el controlador del robot no ha comenzado a funcionar.
        
# Actualizacion de posiciones:  
    def odom_callback(self, odom_msg): #Definicion del metodo.(actualiza la posicion actual del Robot con coordenadas y su orientacion)
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
        # Convierte el cuaternión de orientación a ángulos de Euler
        (
            self.odom_orientation_roll,
            self.odom_orientation_pitch,
            self.odom_orientation_yaw,
        ) = euler_from_quaternion(orientation_list)
 
# Calcula los comandos de velocidad que se enviarán al robot para moverlo hacia un objetivo específico
    def move_to_target(self):
        dx = self.target_x - self.odom_position_x
        dy = self.target_y - self.odom_position_y
        distance_to_target = (dx**2 + dy**2) ** 0.5 # Calcula la distancia euclidiana entre la posición actual del robot y el objetivo
        cmd_vel_msg = Twist() # Iniciacion mensae a robot, envia comandos de velocidad.
        if distance_to_target <= 1.0:
            cmd_vel_msg.linear.x = 0.0
            self.started = False  # CAMBIAR EN EL OTRO
        else:
            cmd_vel_msg.linear.x = min(self.k_linear * distance_to_target, 0.1)
        self.cmd_vel_pub.publish(cmd_vel_msg)
   
# Calcula y envía comandos de velocidad al robot para que gire hacia un ángulo objetivo.
    def rotate_to_target(self):
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
           
            self.cmd_vel_pub.publish(cmd_vel_msg)
            
# Controla el movimiento del robot según las solicitudes recibidas a través del servicio.
    def move_robot_callback(self, req):
        self.started = True
        while not rospy.is_shutdown():
            if self.started: # Comprueba si se ha inicializado el movimiento.
                if req.Rotacion0 == 1: # Se ha solicitado un movimiento lineal.
                    self.target_x = req.Distancia # Indica la distancia que debe de moverse.
                    self.move_to_target() # Calcular y enviar los comandos de velocidad necesarios al robot para que se mueva hacia el objetivo.
                elif req.Rotacion0 == 0: # Se ha solicitado un movimiento de rotacion.
                    # Transforma la entrada en grados sexagesimales a radianes
                    self.target_z = req.Distancia * (math.pi)/180
                    # El ángulo objetivo es el ángulo actual (ángulo yaw) más el ángulo de giro pedido
                    # self.target_z += self.odom_orientation_yaw
                    # Inicia la rotación
                    self.rotate_to_target()
            else:
                self.stop()
                break  # cambiar en el otro
            self.rate.sleep()
        return ResponselienalorotationResponse(True)
 
 # Proporciona la parada del movimiento del robot:   
    def stop(self):
        cmd_vel_msg = Twist()
        self.cmd_vel_pub.publish(cmd_vel_msg)

# Mantiene el nodo en ejecución y permitir que continúe recibiendo y respondiendo a mensajes:
    def control_loop(self):
        rospy.spin()

# Verifica si el script se esta ejecutando como programa principal:
if __name__ == '__main__':

    try:
        controller = RobotController() # Verifica si el script se esta eecutando como programa principal
        rospy.Service('move_robot', Responselienalorotation, controller.move_robot_callback) # Define un sevicio
        rospy.loginfo("Servicio de movimiento del robot iniciado.") # Confirmacion sevicio
        controller.control_loop() #Llama la metodo, responsable de controlar el movimiento del robot
    except rospy.ROSInterruptException:
        pass # Finaliza el programa cuando recibe una señal de interrupcion.
 
