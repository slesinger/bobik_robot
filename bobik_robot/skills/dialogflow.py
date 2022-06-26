from google.cloud import dialogflow_v2beta1 as dialogflow
from mpd import MPDClient, ConnectionError
from google.cloud import texttospeech
import socket
import time
from threading import Thread
import pyogg
import pyaudio
import random

UDP_CHUNK = 1024


class DialogFlow:

    def __init__(self, project_id, session_id, language_code):
        self.project_id = project_id
        self.session_id = session_id
        self.language_code = language_code
        self.session_client = dialogflow.SessionsClient()
        self.session = self.session_client.session_path(project_id, session_id)

        self.mpd_client = MPDClient()
        self.mpd_client.timeout = 10
        self.mpd_client.idletimeout = None

        self.tts_client = texttospeech.TextToSpeechClient()
        self.audio = None


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

        self.audio = response.output_audio
        if response.output_audio[:3] == b'RIF':
            self.audio_type = 'wav'
            # new_thread = Thread(target=self.stream_pcm_s16le24k_audio,args=(response.output_audio,))
            # new_thread.start()
        elif response.output_audio[:3] == b'Ogg':
            # MPD is not able to play Opus
            self.audio_type = 'opus'
            # new_thread = Thread(target=self.stream_opus_audio_48k,args=(response.output_audio,))
            # new_thread.start()
        else:
            self.audio_type = 'mp3'

        self.mpd_play()

        return {
            'reply': response.query_result.fulfillment_text, 
            'confidence': response.query_result.intent_detection_confidence,
            'audio_size': len(self.audio),
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

        self.audio = response.output_audio
        if response.output_audio[:3] == b'RIF':
            self.audio_type = 'wav'
        elif response.output_audio[:3] == b'Ogg':
            print('warn: MPD is not able to play Opus')
            self.audio_type = 'opus'
        else:
            self.audio_type = 'mp3'

        self.mpd_play()

        return {
            'reply': response.query_result.fulfillment_text, 
            'confidence': response.query_result.intent_detection_confidence,
            'audio_size': len(response.output_audio),
            'df_time': int((end - start) * 1000),
            'audio': response.output_audio
        }

    def mpd_play(self):
        try:
            self.mpd_client.status()
        except ConnectionError:
            print('Reconnecting to MPD')
            self.mpd_client.connect("bobik", 6600)
            self.mpd_client.single(1)
            self.mpd_client.add('http://ha.doma:9547/audio')

        self.mpd_client.play(0)


    def mpd_stop(self):
        try:
            self.mpd_client.status()
        except ConnectionError:
            print('Reconnecting to MPD')
            self.mpd_client.connect("bobik", 6600)
            self.mpd_client.single(1)
            self.mpd_client.add('http://ha.doma:9547/audio')

        self.mpd_client.stop()


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
            audio_encoding=texttospeech.AudioEncoding.MP3
        )
        response = self.tts_client.synthesize_speech(
            input=synthesis_input, voice=voice, audio_config=audio_config
        )

        self.audio = response.audio_content
        self.audio_type = 'mp3'
        self.mpd_play()
        return
        