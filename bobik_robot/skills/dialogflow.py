from google.cloud import dialogflow_v2beta1 as dialogflow
#from google.cloud import dialogflow
import socket
import time
from threading import Thread
# from utils.pyogg import OpusDecoder
import pyogg

UDP_CHUNK = 1024

def detect_intent(project_id, session_id, text, language_code):
    """Returns the result of detect intent and confidence from text as input.

    Using the same `session_id` between requests allows continuation
    of the conversation."""

    if not text: return

    session_client = dialogflow.SessionsClient()
    session = session_client.session_path(project_id, session_id)

    text_input = dialogflow.TextInput(text=text, language_code=language_code)
    query_input = dialogflow.QueryInput(text=text_input)

    start = time.time()
    response = session_client.detect_intent(
        request={"session": session, "query_input": query_input}
    )
    end = time.time()

    if response.output_audio[:3] == b'RIF':
        stream_pcm_s16le24k_audio(response.output_audio)
    if response.output_audio[:3] == b'Ogg':
        new_thread = Thread(target=stream_opus_audio_48k,args=(response.output_audio,))
        new_thread.start()
        # stream_opus_audio_48k(response.output_audio)

    return {
        'reply': response.query_result.fulfillment_text, 
        'confidence': response.query_result.intent_detection_confidence,
        'audio_size': len(response.output_audio),
        'df_time': int((end - start) * 1000),
        'action': response.query_result.action,
        'action_params': response.query_result.parameters
    }


# /usr/bin/ffplay rtp://192.168.1.2:7081 -f s16le -ar 24000 -nodisp -loglevel quiet
def stream_pcm_s16le24k_audio(audio_buffer): #TODO play in new thread
    """Streams a plain audio buffer over UDP."""
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = 44
    while True:
        chunk = audio_buffer[start:start+UDP_CHUNK]
        udp.sendto(chunk, ('127.0.0.1', 7081))
        time.sleep(0.02)
        start += UDP_CHUNK
        if start >= len(audio_buffer):
            break

# /usr/bin/ffplay rtp://192.168.1.2:7081 -f s16le -ar 48000 -nodisp -loglevel quiet
def stream_opus_audio_48k(audio_buffer): #TODO play in new thread
    """Decode OGG Opus and streams over UDP."""
    opus_file = pyogg.OpusFile(audio_buffer, len(audio_buffer))
    decoded_pcm = opus_file.buffer
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = 0
    while True:
        mv = memoryview(decoded_pcm).tobytes()
        chunk = mv[start:start+UDP_CHUNK]
        udp.sendto(chunk, ('127.0.0.1', 7081))
        time.sleep(0.01)
        start += UDP_CHUNK
        if start >= len(decoded_pcm):
            break






