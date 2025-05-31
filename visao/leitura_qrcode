import cv2
import qrcode

webcam = cv2.VideoCapture(0)
detector = cv2.QRCodeDetector()

for i in range(1, 4):
    img = qrcode.make(str(i))
    img.save(f"qr_{i}.png")

if webcam.isOpened():
    validacao, frame = webcam.read()
    while validacao: 
        validacao, frame = webcam.read()

        valor, pontos, qrcode = detector.detectAndDecode(frame)

        if valor != "":

            x1 = pontos[0][0][0]
            y1 = pontos[0][0][1]
            x2 = pontos[0][2][1]
            y2 = pontos[0][2][1]

            cv2.putText(frame, str(valor), (20,35), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,0),4)
        
        cv2.imshow("Video da webcam", frame)

        key = cv2.waitKey(5)
        if key == 27: #ESC(Teclas tem um número específico)
            break 

webcam.release()
cv2.destroyAllWindows()



