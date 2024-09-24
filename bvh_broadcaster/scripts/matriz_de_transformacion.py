import rospy
import tf
import tf.transformations as tft

def get_spine3_angles():
    rospy.init_node('get_transformations_node')
    listener = tf.TransformListener()

    try:
        #Esperemos que el frame este disponible
        listener.waitForTransform("/world", "/b_l_forearm", rospy.Time(), rospy.Duration(4.0))

        #Obtenemos la transformacion entre 'world' y 'b_spine3'
        (trans, rot) = listener.lookupTransform("/world", "b_l_forearm", rospy.Time())

        roll, pitch, yaw = tft.euler_from_quaternion(rot)

        #Imprimimos
        print(trans)
        print("---------------------")
        print(rot)
        print("---------------------")
        #Imprimimos los resultados
        rospy.loginfo("Roll: {}, Pitch: {}, Yaw: {}".format(roll, pitch, yaw))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Error al obtener la transformacion")

if __name__ == '__main__':
    get_spine3_angles()