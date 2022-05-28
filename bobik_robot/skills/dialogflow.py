from google.cloud import dialogflow_v2beta1 as dialogflow
import socket
import time
# from utils.pyogg import OpusDecoder
import pyogg

UDP_CHUNK = 1024

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
    print(response.output_audio[:3])
    if response.output_audio[:3] == b'RIF':
        stream_pcm_s16le24k_audio(response.output_audio)
    if response.output_audio[:3] == b'Ogg':
        stream_opus_audio_48k(response.output_audio)

    return {
        'reply': response.query_result.fulfillment_text, 
        'confidence': response.query_result.intent_detection_confidence,
        'audio_size': len(response.output_audio),
        'df_time': int((end - start) * 1000),
        'action': response.query_result.action,
        'action_params': response.query_result.parameters
    }


# /usr/bin/ffplay rtp://192.168.1.2:7081 -f s16le -ar 24000 -reorder_queue_size 0 -nodisp -loglevel quiet
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

def stream_opus_audio_48k(audio_buffer): #TODO play in new thread
    """Decode OGG Opus and streams over UDP."""
    opus_file = pyogg.OpusFile(audio_buffer, len(audio_buffer))
    print("Channels:\n  ", opus_file.channels)
    print("Frequency (samples per second):\n  ",opus_file.frequency)
    print("Buffer Length (bytes):\n  ", len(opus_file.buffer))
    decoded_pcm = opus_file.buffer
    # opus_decoder = OpusDecoder()
    # opus_decoder.set_channels(1)
    # opus_decoder.set_sampling_frequency(24000)
    # decoded_pcm = opus_decoder.decode(bytearray(audio_buffer))
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = 0
    while True:
        mv = memoryview(decoded_pcm).tobytes()
        # save memoryview to file
        chunk = mv[start:start+UDP_CHUNK]
        with open('/tmp/test'+str(start)+'.raw', 'wb') as f:
            f.write(chunk)
        udp.sendto(chunk, ('127.0.0.1', 7081))
        time.sleep(0.02)
        start += UDP_CHUNK
        if start >= len(decoded_pcm):
            break






