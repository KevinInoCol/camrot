import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
import tf.transformations as tft

alpha = 0.9

def tf_callback(msg):
    #El mensaje msg contiene una lista de transformaciones
    for transform in msg.transforms:
        #Coloco el frame del cual me interesa extraer los angulos
        if transform.header.frame_id == "b_l_arm":
            
            #Extraemos los quaternions de la transformacion -------------------------------------------------
            q = transform.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]

            #Convertimos el cuaternion a angulos de Euler(Roll, Pitch, Yaw)
            roll, pitch, yaw = tft.euler_from_quaternion(quaternion)

            roll_interpolated = alpha * roll + (1 - alpha) * yaw

            #Imprimimos los valores de Roll y Pitch resultantes
            rospy.loginfo("Roll: {}, Pitch: {}".format(roll_interpolated, pitch))

            publish_joint_trajectory(roll_interpolated, pitch)

def publish_joint_trajectory(roll, pitch):
    pub = rospy.Publisher('/pepper/LeftArm_controller/command', JointTrajectory, queue_size=10)
    traj_msg = JointTrajectory()

    #Nombres de las articulaciones del brazo izquierdo
    traj_msg.joint_names = ["LElbowRoll", "LElbowYaw", "LShoulderPitch", "LShoulderRoll", "LWristYaw"]

    point = JointTrajectoryPoint()
    point.positions = [0, 0, pitch, roll, 0]
    

    #Anadimos el punto a la trayectoria
    traj_msg.points = [point]

    #Publicamos el mensaje
    pub.publish(traj_msg)
    rospy.loginfo("Publicando Roll:{} y Pitch: {}".format(roll , pitch))

def listener():
    #Inicializamos el nodo de ROS
    rospy.init_node('tf_listener_v3')

    #Nos suscribimos al topico /tf
    rospy.Subscriber('/tf', TFMessage, tf_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()