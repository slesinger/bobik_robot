import time
import datetime
from google.cloud import dialogflow_v2beta1 as dialogflow

LC = 'cs-CZ'


def action_kolik_je_hodin(node, params):
    # print(params.get("param2"))
    # new_thread = Thread(target=func,args=(reply.get('action_params'),))
    # new_thread.start()
    hhmm = datetime.datetime.now().strftime("%H:%M")
    return dialogflow.types.EventInput(
        name='event_kolik_je_hodin', 
        language_code=LC, 
        parameters={"hhmm": hhmm})


def action_zmlkni(node, params):
    node.df.mpd_stop()

def action_rozsvit(node, params):
    print('jsem v action_rozsvit')
