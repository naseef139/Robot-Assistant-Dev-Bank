import cv2
import numpy as np
import requests
import io
import json
from gtts import gTTS
import os

img = cv2.imread("tanguin.jpg")
height, width, _ = img.shape

# Cutting image
# roi = img[0: height, 400: width]
roi = img

# Ocr
url_api = "https://api.ocr.space/parse/image"
_, compressedimage = cv2.imencode(".jpg", roi, [1, 90])
file_bytes = io.BytesIO(compressedimage)

result = requests.post(url_api,
              files = {"tanguin.jpg": file_bytes},
              data = {"apikey": "helloworld",
                      "language": "eng"})

result = result.content.decode()
result = json.loads(result)

parsed_results = result.get("ParsedResults")[0]
text_detected = parsed_results.get("ParsedText")
print(text_detected)

cv2.imshow("roi", roi)
cv2.imshow("Img", img)


language = 'en'
speech = gTTS(text = str(text_detected), lang = language, slow = False)
speech.save("voice.mp3")
os.system("start voice.mp3")

cv2.destroyAllWindows()
