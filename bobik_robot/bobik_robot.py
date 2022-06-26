import rclpy
from rclpy.node import Node
import asyncio
from aiohttp import web, MultipartWriter
from threading import Thread
from bobik_interfaces.srv import HumanQuery
from skills import dialogflow, dialogflow_actions

PORT=9547
node = None

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
        self.audio_to_play = None

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
                    new_thread = Thread(target=func,args=(self, reply.get('action_params'),))
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


async def ros2_task(app):
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        print('Keyboard interrupt')
    except asyncio.CancelledError as e:
        print('Cancelling Web+ROS loop')


async def cleanup_background_tasks(app):
    print('Shutting down')
    rclpy.shutdown()

async def api_get_index(request):
    return web.Response(text='<h1>Bobik the Robot</h1><p><a href="/audio">Current audio to play</a></p>', content_type='text/html')

async def api_get_audio(request):
    if node.df.audio != None:
        return web.Response(
            headers={'Content-Disposition': f'attachment;filename=audio.{node.df.audio_type}'},
            body=node.df.audio, 
            status=200, 
            content_type='application/octet-stream'
        )
    else:
        return web.Response(status=204)

async def main():
    global node
    node = BobikRobot()
    app = web.Application()
    app.router.add_route('GET', "/", api_get_index)
    app.router.add_route('GET', "/audio", api_get_audio)
    app.on_cleanup.append(cleanup_background_tasks)

    await asyncio.gather(
        web._run_app(app, port=PORT),
        ros2_task(app)
    )

if __name__ == '__main__':
    asyncio.run(main())
