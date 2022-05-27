from google.cloud import dialogflow_v2beta1 as dialogflow
import socket
import time

def detect_intent(project_id, session_id, text, language_code):
    """Returns the result of detect intent and confidence from text as input.

    Using the same `session_id` between requests allows continuation
    of the conversation."""
    from google.cloud import dialogflow

    session_client = dialogflow.SessionsClient()
    session = session_client.session_path(project_id, session_id)

    text_input = dialogflow.TextInput(text=text, language_code=language_code)
    query_input = dialogflow.QueryInput(text=text_input)

    start = time.time()
    response = session_client.detect_intent(
        request={"session": session, "query_input": query_input}
    )
    end = time.time()
    stream_audio(response.output_audio)  # PCM s16 24k

    return {
        'reply': response.query_result.fulfillment_text, 
        'confidence': response.query_result.intent_detection_confidence,
        'audio_size': len(response.output_audio),
        'df_time': int((end - start) * 1000),
        'action': response.query_result.action,
        'action_params': response.query_result.parameters
    }


# /usr/bin/ffplay rtp://192.168.1.2:7081 -f s16le -ar 24000 -reorder_queue_size 0 -nodisp -loglevel quiet
def stream_audio(audio_buffer): #TODO play in new thread
    """Streams an audio buffer as RTP packets over UDP."""
    CHUNK = 1024
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = 44
    while True:
        chunk = audio_buffer[start:start+CHUNK]
        udp.sendto(chunk, ('127.0.0.1', 7081))
        time.sleep(0.02)
        start += CHUNK
        if start >= len(audio_buffer):
            break






