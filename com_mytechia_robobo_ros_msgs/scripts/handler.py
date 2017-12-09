import rospy

from com_mytechia_robobo_ros_msgs.srv import Command

from com_mytechia_robobo_ros_msgs.msg import KeyValue


class Handler():


    def __init__(self):
        self.actions={}
        self.command_id=0;



    def send_command(self, command_name, parameters):

        rospy.init_node('service_client')

        rospy.wait_for_service('command', 1)

        command_proxy = rospy.ServiceProxy('command', Command)


        command_parameters= []

        for key in parameters:
            key_value= KeyValue()
            key_value.key= key
            key_value.value= parameters[key]
            command_parameters.append(key_value)

        print "Enviando Comando: nombre:{}, id:{}, parameters:{}".format(command_name, self.command_id, parameters)

        command_proxy(command_name, self.command_id, command_parameters)

        self.command_id= self.command_id + 1;





