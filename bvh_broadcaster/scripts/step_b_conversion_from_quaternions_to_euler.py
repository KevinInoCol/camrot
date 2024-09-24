import rospy
from tf2_msgs.msg import TFMessage
import tf.transformations as tft

def tf_callback(msg):
    #El mensaje msg contiene una lista de transformaciones
    for transform in msg.transforms:
        #Coloco el frame del cual me interesa extraer los angulos
        if transform.header.frame_id == "b_l_arm":
            
            #Extraemos los quaternions de la transformacion -------------------------------------------------
            q = transform.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]

            #Convetimos el quaternion original a una matriz de rotacion ------------------------------------
            rotation_matrix = tft.quaternion_matrix(quaternion)

            #Eliminamos la componente Yaw (rotacion alrededor del eje Z) -----------------------------------
            #Modificamos la matriz de rotacion para eliminar la rotacion alrededor del eje Z
            rotation_matrix[2, 0:3] = [0, 0, 1]

            #Convertimos la matriz modificada de nuevo a un quaternion ------------------------------------
            quaternion_reduced = tft.quaternion_from_matrix(rotation_matrix)

            #Convertimos el cuaternion a angulos de Euler(Roll, Pitch, Yaw)
            roll_reduced, pitch_reduced, _ = tft.euler_from_quaternion(quaternion_reduced)

            #Imprimimos los valores de Roll y Pitch resultantes
            rospy.loginfo("Roll: {}, Pitch: {}".format(roll_reduced, pitch_reduced))

def listener():
    #Inicializamos el nodo de ROS
    rospy.init_node('tf_listener_v2')

    #Nos suscribimos al topico /tf
    rospy.Subscriber('/tf', TFMessage, tf_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

#python step_a_extraction_of_quaternions.py