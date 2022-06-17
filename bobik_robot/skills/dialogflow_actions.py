import time
import datetime
from google.cloud import dialogflow_v2beta1 as dialogflow

LC = 'cs-CZ'


def action_kolik_je_hodin(params):
    # print(params.get("param2"))
    # new_thread = Thread(target=func,args=(reply.get('action_params'),))
    # new_thread.start()
    hhmm = datetime.datetime.now().strftime("%H:%M")
    return dialogflow.types.EventInput(
        name='event_kolik_je_hodin', 
        language_code=LC, 
        parameters={"hhmm": hhmm})
