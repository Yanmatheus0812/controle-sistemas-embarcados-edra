import numpy as np
import cv2

# Faixa de azul em HSV (ajuste se necessário)
azulEscuro = np.array([100, 150, 50])
azulClaro  = np.array([140, 255, 255])

camera = cv2.VideoCapture(0)

while True:
    sucesso, frame = camera.read()
    if not sucesso:
        break

    altura, largura, _ = frame.shape

    # Região de interesse: parte inferior da imagem
    roi = frame[int(0.7 * altura):, :]

    # Converte para HSV
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Máscara para cor azul
    obj = cv2.inRange(hsv, azulEscuro, azulClaro)
    obj = cv2.GaussianBlur(obj, (3, 3), 0)

    # Calcula o centro da linha com momentos
    M = cv2.moments(obj)

    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Desenha ponto no centro da linha
        cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)

        # Linha central da imagem
        cv2.line(roi, (largura // 2, 0), (largura // 2, roi.shape[0]), (255, 255, 0), 2)

        # Linha verde mostrando o desvio
        cv2.line(roi, (largura // 2, cy), (cx, cy), (0, 255, 0), 2)

        # Calcula o erro (desvio do centro)
        erro = cx - (largura // 2)
        print(f"Erro de posição: {erro} px")

    # Exibe os resultados
    cv2.imshow("Tracking", roi)
    cv2.imshow("Binary", obj)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()
