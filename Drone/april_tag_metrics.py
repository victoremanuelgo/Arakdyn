import cv2
import numpy as np
import apriltag
import time
from djitellopy import Tello

# Definindo o tamanho da largura (w) e altura (h) da imagem
w, h = 500, 500

# Definindo o intervalo de áreas para detecção da April Tag
fbRange = [1500, 3000]

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
        ["VIRAR ANTI-HORÁRIO", "CENTRALIZADO", "CENTRALIZADO", "CENTRALIZADO", "VIRAR HORÁRIO"],
        ["VIRAR ANTI-HORÁRIO", "CENTRALIZADO", "CENTRALIZADO", "CENTRALIZADO", "VIRAR HORÁRIO"],
        ["VIRAR ANTI-HORÁRIO", "DESCER", "DESCER", "DESCER", "VIRAR HORÁRIO"]
    ]

    return actions[row][col]

# Função para ajustar a orientação do drone com base na posição da April Tag
def turn_drone(action):
    yaw = 0
    if action == "VIRAR ANTI-HORÁRIO":
        yaw = -30
    elif action == "VIRAR HORÁRIO":
        yaw = 30
    return yaw

# Função para determinar a subida/descida do drone com base na ação
def vertical_movement(action):
    up_down = 0
    if action == "SUBIR":
        up_down = 20
    elif action == "DESCER":
        up_down = -20
    return up_down

# Função para rastrear a April Tag
def track_april_tag(info, w, h):
    area = info[1]
    cX, cY = info[0]
    section_width = w // 5

    # Movimento para frente e para trás
    fb = 0
    if area > fbRange[0] and area < fbRange[1]:
        fb = 0
    elif area > fbRange[1]:
        fb = -30
    elif area < fbRange[0] and area != 0:
        fb = 30
    print("Frente/Trás:", fb)

    # Determinar a ação com base na posição
    action = determine_action(cX, cY, w, h)
    print(f"Ação: {action}")

    # Virando com base na posição
    yaw = turn_drone(action)
    print(f"Yaw: {yaw}")

    # Subida/descida com base na posição
    up_down = vertical_movement(action)
    print(f"Subida/Descida: {up_down}")

    return fb, yaw, up_down

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
me.takeoff()
time.sleep(2.0)
# Alterando a altura inicial
# me.send_rc_control(0, 0, 24, 0)
# time.sleep(1.0)

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

            fb, yaw, up_down = track_april_tag([april_tag_info, area], w, h)
            me.send_rc_control(0, fb, up_down, yaw)
        else:
            # Para o drone se a April Tag não for detectada
            me.send_rc_control(0, 0, 0, 0)

        processing_time = time.time() - start_time
        total_processing_time += processing_time

        draw_grid(img_bgr_resized, w, h)
        cv2.imshow("Imagem", img_bgr_resized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            me.land()
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
