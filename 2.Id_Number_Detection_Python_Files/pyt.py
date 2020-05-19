from gtts import gTTS
import os
import cv2
import pytesseract
import re
pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'


#img = cv2.imread()

cap = cv2.VideoCapture(0)

def gray_scale(image):
	return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


while(True):
	ret, frame = cap.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	cv2.imshow('frame', gray)

	text = pytesseract.image_to_string(gray)
	result = re.match(r'\d\d\d\d\d\d\d\d\d', text)

	if result:
		break

	if cv2.waitKey(1) & 0xFF == ord('q'):
		print('ok done......')



language = 'en'
speech = gTTS(text = str("its ok thank you very much and have a nice day"), lang = language, slow = False)
speech.save("voice.mp3")
os.system("start voice.mp3")
print(text)



cv2.destroyAllWindows()

#gray = gray_scale(img)
