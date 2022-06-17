import rclpy
from rclpy.node import Node
from threading import Thread
from bobik_interfaces.srv import HumanQuery
from skills import dialogflow, dialogflow_actions

class BobikRobot(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('bobik_robot')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.srv = self.create_service(HumanQuery, 'human_query', self.human_query_callback)
        self.srv = self.create_service(HumanQuery, 'bobik_say', self.bobik_say_callback)
        self.srv = self.create_service(HumanQuery, 'bobik_poweroff', self.bobik_poweroff_callback)
        self.srv = self.create_service(HumanQuery, 'motor_relay', self.motor_relay_callback)

        self.df = dialogflow.DialogFlow('robik-d2d55', 'bobik_robot', 'cs-CZ')

    def human_query_callback(self, request, response):
        query = request.query
        reply = self.df.detect_intent(
            text=query
        )
        if not reply:
            response.reply = "<i>žádné zadání<i>"
            return response

        response.reply = reply.get('reply')
        self.get_logger().info('Human query: %s >> %s (conf: %d, size: %d, millis %d)' % (query, reply.get('reply'), reply.get('confidence'), reply.get('audio_size'), reply.get('df_time')))
        if reply.get('action'):
            try:
                func = getattr(dialogflow_actions, reply.get('action'))
                if reply.get('reply'):
                    self.get_logger().info('Invoking async action: %s ' % reply.get('action'))
                    new_thread = Thread(target=func,args=(reply.get('action_params'),))
                    new_thread.start()
                else:
                    self.get_logger().info('Exec action: %s ' % reply.get('action'))
                    event_reply = self.df.detect_event(
                        event=func(reply.get('action_params'))
                    )
                    response.reply = event_reply.get('reply')
                    self.get_logger().info('Human query on event: %s >> %s (conf: %d, size: %d, millis %d)' % (query, event_reply.get('reply'), event_reply.get('confidence'), event_reply.get('audio_size'), event_reply.get('df_time')))
            except AttributeError:
                self.get_logger().info('Action not found: %s ' % reply.get('action'))
        return response  # Response visible in Web UI

    def bobik_say_callback(self, request, response):
        self.df.tts_play(request.query)
        return response

    def bobik_poweroff_callback(self, request, response):
        self.df.tts_play("Vypínám se")
        return response

    def motor_relay_callback(self, request, response):
        if request.query == "poweron":
            self.df.tts_play("Zapínám motory")
        else:
            self.df.tts_play("Vypínám motory")
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
