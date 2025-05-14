import speech_recognition as sr
import socket

recognizer = sr.Recognizer()
mic = sr.Microphone()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(("127.0.0.1", 5000))
    print("Connected to WSL DroneController. Speak commands.")

    while True:
        try:
            with mic as source:
                print("Say a command...")
                audio = recognizer.listen(source)

            # First recognize the audio
            command = recognizer.recognize_google(audio)

            # Then fix common misheard words
            command = command.lower()

            print(f"You said: {command}")
            s.sendall(command.encode())

            if "exit" in command:
                print("Landing command received. Exiting...")
                break

        except sr.UnknownValueError:
            print("Didn't catch that. Try again.")
        except Exception as e:
            print(f"Error: {e}")
