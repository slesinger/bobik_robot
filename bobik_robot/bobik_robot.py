import rclpy
from rclpy.node import Node
from bobik_interfaces.srv import HumanQuery
from skills import dialogflow

class BobikRobot(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('bobik_robot')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.srv = self.create_service(HumanQuery, 'human_query', self.human_query_callback)

    def human_query_callback(self, request, response):
        query = request.query
        reply = dialogflow.detect_intent(
            project_id='robik-d2d55',
            session_id='bobik-robot',
            text=query,
            language_code='cs-CZ'
        )
        response.reply = reply.get('reply')
        self.get_logger().info('Human query: %s >> %s (conf: %d, size: %d, millis %d)' % (query, reply.get('reply'), reply.get('confidence'), reply.get('audio_size'), reply.get('df_time')))
        return response


def main():
    node = BobikRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
