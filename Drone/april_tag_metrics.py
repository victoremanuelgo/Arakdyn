import cv2
import numpy as np
import apriltag
import time
from djitellopy import Tello

# Definindo o tamanho da largura (w) e altura (h) da imagem
w, h = 360, 240

# Inicializando o detector de April Tags
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

# Variáveis para métricas
total_frames = 0
detected_frames = 0
false_positives = 0
total_position_error = 0
total_processing_time = 0

# Função para detectar April Tags na imagem
def detect_april_tag(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)
    if results:
        r = results[0]  # Considerando apenas a primeira tag detectada
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        area = (ptB[0] - ptA[0]) * (ptD[1] - ptA[1])
        return (cX, cY), area, (ptA, ptB, ptC, ptD)
    return None, 0, None

# Função para determinar a ação com base na posição do quadrante
def determine_action(cX, cY, width, height):
    grid_rows, grid_cols = 5, 5
    cell_width = width // grid_cols
    cell_height = height // grid_rows

    col = cX // cell_width
    row = cY // cell_height

    actions = [
        ["VIRAR ANTI-HORÁRIO", "SUBIR", "SUBIR", "SUBIR", "VIRAR HORÁRIO"],
        ["VIRAR ANTI-HORÁRIO", "SUBIR", "SUBIR", "SUBIR", "VIRAR HORÁRIO"],
        ["VIRAR ANTI-HORÁRIO", "OK NÃO MEXER", "OK NÃO MEXER", "OK NÃO MEXER", "VIRAR HORÁRIO"],
        ["VIRAR ANTI-HORÁRIO", "OK NÃO MEXER", "OK NÃO MEXER", "OK NÃO MEXER", "VIRAR HORÁRIO"],
        ["VIRAR ANTI-HORÁRIO", "DESCER", "DESCER", "DESCER", "VIRAR HORÁRIO"]
    ]

    return actions[row][col]

# Função para executar a ação
def execute_action(action, me):
    if action == "SUBIR":
        me.move_up(15)
    elif action == "DESCER":
        me.move_down(15)
    elif action == "VIRAR HORÁRIO":
        me.rotate_clockwise(20)
    elif action == "VIRAR ANTI-HORÁRIO":
        me.rotate_counter_clockwise(20)

# Função para desenhar a grade na imagem
def draw_grid(frame, width, height):
    grid_rows, grid_cols = 5, 5
    cell_width = width // grid_cols
    cell_height = height // grid_rows

    for i in range(1, grid_rows):
        cv2.line(frame, (0, i * cell_height), (width, i * cell_height), (0, 255, 0), 1)
    for j in range(1, grid_cols):
        cv2.line(frame, (j * cell_width, 0), (j * cell_width, height), (0, 255, 0), 1)

# Inicializando o drone
me = Tello()
me.connect()
print(me.get_battery())
me.streamon()

# Loop principal para detecção e rastreamento contínuos
while True:
    try:
        frame = me.get_frame_read().frame

        # Convertendo a imagem para o formato correto
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        start_time = time.time()
        total_frames += 1

        img_bgr_resized = cv2.resize(frame, (w, h))
        april_tag_info, area, corners = detect_april_tag(img_bgr_resized, detector)
        if april_tag_info is not None:
            detected_frames += 1
            (cX, cY) = april_tag_info
            (ptA, ptB, ptC, ptD) = corners
            cv2.line(img_bgr_resized, ptA, ptB, (0, 255, 0), 2)
            cv2.line(img_bgr_resized, ptB, ptC, (0, 255, 0), 2)
            cv2.line(img_bgr_resized, ptC, ptD, (0, 255, 0), 2)
            cv2.line(img_bgr_resized, ptD, ptA, (0, 255, 0), 2)
            cv2.circle(img_bgr_resized, (cX, cY), 5, (0, 0, 255), -1)
            
            action = determine_action(cX, cY, w, h)
            print(f"Ação: {action}")
            execute_action(action, me)
            print()  # Linha em branco para separar as saídas

            # Simulando a posição real (por exemplo, centro da imagem)
            real_position = (w // 2, h // 2)
            position_error = np.sqrt((cX - real_position[0]) ** 2 + (cY - real_position[1]) ** 2)
            total_position_error += position_error

        else:
            false_positives += 1

        processing_time = time.time() - start_time
        total_processing_time += processing_time

        draw_grid(img_bgr_resized, w, h)
        cv2.imshow("Imagem", img_bgr_resized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"[ERROR] {e}")

me.streamoff()
cv2.destroyAllWindows()

# Cálculo das métricas
detection_rate = detected_frames / total_frames * 100
false_positive_rate = false_positives / total_frames * 100
average_position_error = total_position_error / detected_frames if detected_frames else 0
average_processing_time = total_processing_time / total_frames

# Exibição das métricas
print("\nMétricas de Avaliação:")
print(f"Taxa de Detecção: {detection_rate:.2f}%")
print(f"Taxa de Falsos Positivos: {false_positive_rate:.2f}%")
print(f"Erro Médio de Posição: {average_position_error:.2f} pixels")
print(f"Tempo Médio de Processamento: {average_processing_time:.4f} segundos")

