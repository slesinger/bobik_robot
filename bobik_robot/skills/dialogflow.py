from google.cloud import dialogflow_v2beta1 as dialogflow
from google.cloud import texttospeech
import socket
import time
from threading import Thread
import pyogg

UDP_CHUNK = 1024


class DialogFlow:

    def __init__(self, project_id, session_id, language_code):
        self.project_id = project_id
        self.session_id = session_id
        self.language_code = language_code
        self.session_client = dialogflow.SessionsClient()
        self.session = self.session_client.session_path(project_id, session_id)

        self.tts_client = texttospeech.TextToSpeechClient()


    def detect_intent(self, text):
        """Returns the result of detect intent and confidence from text as input.
        Using the same `session_id` between requests allows continuation
        of the conversation."""

        if not text: return

        text_input = dialogflow.TextInput(text=text, language_code=self.language_code)
        query_input = dialogflow.QueryInput(text=text_input)
        start = time.time()
        response = self.session_client.detect_intent(
            request={"session": self.session, "query_input": query_input}
        )
        end = time.time()

        if response.output_audio[:3] == b'RIF':
            new_thread = Thread(target=self.stream_pcm_s16le24k_audio,args=(response.output_audio,))
            new_thread.start()
        if response.output_audio[:3] == b'Ogg':
            new_thread = Thread(target=self.stream_opus_audio_48k,args=(response.output_audio,))
            new_thread.start()

        return {
            'reply': response.query_result.fulfillment_text, 
            'confidence': response.query_result.intent_detection_confidence,
            'audio_size': len(response.output_audio),
            'df_time': int((end - start) * 1000),
            'action': response.query_result.action,
            'action_params': response.query_result.parameters
        }


    def detect_event(self, event):
        """This method is called after DF responded with actiion which was processed by
        Bobik action handler and returned event.
        Further action is disallowed."""

        if not event: return

        query_input = dialogflow.QueryInput(event=event)
        start = time.time()
        response = self.session_client.detect_intent(
            request={"session": self.session, "query_input": query_input}
        )
        end = time.time()

        if response.output_audio[:3] == b'RIF':
            new_thread = Thread(target=self.stream_pcm_s16le24k_audio,args=(response.output_audio,))
            new_thread.start()
        if response.output_audio[:3] == b'Ogg':
            new_thread = Thread(target=self.stream_opus_audio_48k,args=(response.output_audio,))
            new_thread.start()

        return {
            'reply': response.query_result.fulfillment_text, 
            'confidence': response.query_result.intent_detection_confidence,
            'audio_size': len(response.output_audio),
            'df_time': int((end - start) * 1000)
        }


    def tts_play(self, text):
        """Convert text to speech and play on Bobik"""

        if not text: return
        synthesis_input = texttospeech.SynthesisInput(text=text)
        voice = texttospeech.VoiceSelectionParams(
            language_code="cs-CZ", name="cs-CZ-Wavenet-A"
        )
        audio_config = texttospeech.AudioConfig(
            speaking_rate=0.85,
            pitch=-12.0,
            audio_encoding=texttospeech.AudioEncoding.OGG_OPUS
        )
        response = self.tts_client.synthesize_speech(
            input=synthesis_input, voice=voice, audio_config=audio_config
        )

        new_thread = Thread(target=self.stream_opus_audio_48k,args=(response.audio_content,))
        new_thread.start()
        return


    # /usr/bin/ffplay rtp://192.168.1.2:7081 -f s16le -ar 24000 -nodisp -loglevel quiet
    def stream_pcm_s16le24k_audio(self, audio_buffer): #TODO play in new thread
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
    # gst-launch-0.10 udpsrc port=7081 ! audio/x-raw-int, endianness="(int)1234", signed="(boolean)true", width="(int)16", depth="(int)16", rate="(int)48000", channels="(int)1" ! alsasink
    def stream_opus_audio_48k(self, audio_buffer): #TODO play in new thread
        """Decode OGG Opus and streams over UDP."""
        opus_file = pyogg.OpusFile(audio_buffer, len(audio_buffer))
        decoded_pcm = opus_file.buffer
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        start = 0
        while True:
            mv = memoryview(decoded_pcm).tobytes()
            chunk = mv[start:start+UDP_CHUNK]
            udp.sendto(chunk, ('192.168.1.21', 7081))
            time.sleep(0.01)
            start += UDP_CHUNK
            if start >= len(decoded_pcm):
                break
