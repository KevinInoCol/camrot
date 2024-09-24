import rospy
from tf2_msgs.msg import TFMessage
import tf.transformations as tft

def tf_callback(msg):
    #El mensaje msg contiene una lista de transformaciones
    for transform in msg.transforms:
        #Coloco el frame del cual me interesa extraer los angulos
        if transform.header.frame_id == "b_l_forearm":
            
            #Extraemos los quaternions de la transformacion
            q = transform.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]

            #Convertimos el cuaternion a angulos de Euler(Roll, Pitch, Yaw)
            (roll, pitch, yaw) = tft.euler_from_quaternion(quaternion)

            #Imprimimos los resultados
            rospy.loginfo("Roll: {}, Pitch: {}, Yaw: {}".format(roll, pitch, yaw))

def listener():
    #Inicializamos el nodo de ROS
    rospy.init_node('tf_listener')

    #Nos suscribimos al topico /tf
    rospy.Subscriber('/tf', TFMessage, tf_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

#python step_a_extraction_of_quaternions.py