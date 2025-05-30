import numpy as np
import cv2

# Faixa de tons de azul a rastrear
azulEscuro = np.array([100, 67, 0], dtype="uint8")
azulClaro  = np.array([255, 128, 50], dtype="uint8")

# Inicia a webcam
camera = cv2.VideoCapture(0)

while True:
    sucesso, frame = camera.read()
    if not sucesso:
        break

    # Cria imagem binária com apenas os tons de azul
    obj = cv2.inRange(frame, azulEscuro, azulClaro)
    obj = cv2.GaussianBlur(obj, (3, 3), 0)

    # Detecta contornos
    cnts, _ = cv2.findContours(obj.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts) > 0:
        # Pega o maior contorno
        cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
        rect = np.int32(cv2.boxPoints(cv2.minAreaRect(cnt)))
        cv2.drawContours(frame, [rect], -1, (0, 255, 255), 2)

    # Mostra as imagens
    cv2.imshow("Tracking", frame)
    cv2.imshow("Binary", obj)

    # Encerra se apertar 'q'
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Finaliza
camera.release()
cv2.destroyAllWindows()
