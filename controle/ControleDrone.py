from dronekit import connect, VehicleMode, LocationGlobalRelative, Attitude
from pymavlink import mavutil # Necessário para set_velocity_body
import time
import math

SERIAL_DEVICE = '/dev/ttyAMA0' # pode ser necessário desabilitar o Bluetooth
                               # (adicionando dtoverlay=disable-bt em /boot/config.txt ou /boot/firmware/config.txt e reiniciando)
                                
SERIAL_BAUDRATE = 57600 # alternativas caso dê erro: 115200, 230400, 460800, 921600. 

CHOSEN_CONNECTION_DEVICE = SERIAL_DEVICE
CHOSEN_BAUDRATE = SERIAL_BAUDRATE

# --- Conexão com o Veículo ---
vehicle = None
print(f"Tentando conectar ao veículo via SERIAL em: {CHOSEN_CONNECTION_DEVICE} com baudrate: {CHOSEN_BAUDRATE}")
try:
    # Para conexões seriais, o baudrate é essencial
    vehicle = connect(CHOSEN_CONNECTION_DEVICE,
                      wait_ready=True,
                      baud=CHOSEN_BAUDRATE,
                      timeout=60,  # Tempo limite para a conexão inicial
                      heartbeat_timeout=90) # Tempo limite para heartbeats MAVLink
    print("Veículo conectado via SERIAL!")
    print(f"  Versão do Firmware: {vehicle.version}")
    print(f"  Modo Atual: {vehicle.mode.name}")
    print(f"  Armado: {vehicle.armed}")
    print(f"  GPS: {vehicle.gps_0}")
    print(f"  Bateria: {vehicle.battery}")

except Exception as e:
    print(f"ERRO AO CONECTAR VIA SERIAL: {e}")
    print("\nVERIFICAÇÕES IMPORTANTES PARA CONEXÃO SERIAL NO RASPBERRY PI:")
    print(f"1. CONTROLADOR DE VOO (FC): Está ligado e a porta TELEM conectada aos pinos GPIO corretos do Raspberry Pi?")
    print(f"   (RPi TXD [GPIO14] -> FC RX | RPi RXD [GPIO15] -> FC TX | RPi GND -> FC GND)")
    print(f"2. PORTA SERIAL NO RASPBERRY PI ('{CHOSEN_CONNECTION_DEVICE}'):")
    print(f"   - Está correta? Use 'ls /dev/ttyS*' e 'ls /dev/ttyAMA*' para listar as portas seriais disponíveis.")
    print(f"   - A interface serial está habilitada no Raspberry Pi? (Verifique com 'sudo raspi-config' -> Interface Options -> Serial Port)")
    print(f"     - 'Login shell accessible over serial' deve estar DESABILITADO.")
    print(f"     - 'Serial port hardware' deve estar HABILITADO.")
    print(f"   - Se estiver usando '{SERIAL_DEVICE}' e ele for o /dev/ttyAMA0, pode ser necessário desabilitar o Bluetooth")
    print(f"     (adicione 'dtoverlay=disable-bt' em /boot/config.txt ou /boot/firmware/config.txt e reinicie o RPi).")
    print(f"3. BAUDRATE ({CHOSEN_BAUDRATE}):")
    print(f"   - Corresponde EXATAMENTE ao configurado no parâmetro SERIALx_BAUD do seu controlador de voo para a porta TELEM usada?")
    print(f"4. PROTOCOLO SERIAL NO CONTROLADOR DE VOO:")
    print(f"   - O parâmetro SERIALx_PROTOCOL está configurado para MAVLink 1 ou MAVLink 2?")
    print(f"5. PERMISSÕES NO RASPBERRY PI:")
    print(f"   - O usuário que executa o script pertence ao grupo 'dialout'?")
    print(f"     (Execute 'sudo usermod -a -G dialout $USER', depois FAÇA LOGOUT E LOGIN NOVAMENTE ou reinicie o RPi).")
    print(f"6. CABEAMENTO: Verifique se não há mau contato ou inversão dos fios TX/RX.")
    exit()


def arm_and_takeoff(aTargetAltitude):
    """
    Arma o veículo e decola para aTargetAltitude.
    """
    print("Checagens básicas pré-voo")
    # Espera o veículo se tornar "armável"
    # Adiciona um timeout para não ficar preso indefinidamente se o GPS demorar muito, por exemplo
    timeout_armable = time.time() + 60 # 60 segundos de timeout
    while not vehicle.is_armable:
        if time.time() > timeout_armable:
            print("Timeout: Veículo não se tornou armável em 60 segundos.")
            raise Exception("Falha ao armar: Veículo não se tornou armável a tempo.")
        print(" Aguardando o veículo se tornar armável (requer bom sinal GPS em alguns modos/configurações)...")
        print(f"   GPS: {vehicle.gps_0.fix_type} satellites: {vehicle.gps_0.satellites_visible}")
        print(f"   EKF OK?: {vehicle.ekf_ok}") # Checa o status do EKF (Extended Kalman Filter)
        time.sleep(2)

    print("Armando os motores")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("  Aguardando modo GUIDED...")
        time.sleep(0.5)

    vehicle.armed = True
    # Confirma que o veículo armou
    timeout_armed_confirm = time.time() + 10 # 10 segundos para confirmar o arme
    while not vehicle.armed:
        if time.time() > timeout_armed_confirm:
            print("Timeout: Veículo não confirmou o arme.")
            raise Exception("Falha ao armar: Veículo não confirmou o arme a tempo.")
        print(" Aguardando armar...")
        time.sleep(0.5)

    print(f"Decolando para {aTargetAltitude} metros!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
        if vehicle.location.global_relative_frame.alt is not None and \
           vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Altitude alvo atingida")
            break
        elif vehicle.location.global_relative_frame.alt is None:
            print("Aguardando dados de altitude válidos...")
        time.sleep(1)

# --- Controle de Velocidade ---
def set_velocity_body(vehicle_obj, vx, vy, vz, duration):
   
    print(f"Definindo velocidade: vx={vx}, vy={vy}, vz={vz} por {duration}s")
    msg = vehicle_obj.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (não usado)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (apenas velocidades)
        0, 0, 0, # x, y, z posições (não usado)
        vx, vy, vz, # x, y, z velocidades em m/s
        0, 0, 0, # x, y, z acelerações (não usado)
        0, 0)    # yaw, yaw_rate (não usado)

    start_time = time.time()
    while time.time() - start_time < duration:
        vehicle_obj.send_mavlink(msg)
        time.sleep(0.1)
    stop_msg = vehicle_obj.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    vehicle_obj.send_mavlink(stop_msg)
    print("Comando de velocidade concluído.")

def get_distance_metres(aLocation1, aLocation2):
    """
    Retorna a distância em metros entre duas LocationGlobal objects.
    """
    if aLocation1.lat is None or aLocation1.lon is None or aLocation2.lat is None or aLocation2.lon is None:
        return float('inf')

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# --- Controle de Orientação (Atitude) ---
def set_attitude(vehicle_obj, roll_angle=0.0, pitch_angle=0.0, yaw_angle=None, thrust=0.5, duration=0):
    """
    Define a atitude do veículo.
    
    """
    print(f"Definindo atitude: Roll={roll_angle}, Pitch={pitch_angle}, Yaw={yaw_angle}, Thrust={thrust} por {duration}s")

    roll_rad = math.radians(roll_angle)
    pitch_rad = math.radians(pitch_angle)
    if yaw_angle is not None:
        yaw_rad = math.radians(yaw_angle)
        type_mask = 0b00000000
    else:
        yaw_rad = vehicle_obj.attitude.yaw
        type_mask = 0b00000000

    q = to_quaternion(roll_rad, pitch_rad, yaw_rad)

    msg = vehicle_obj.message_factory.set_attitude_target_encode(
        0, 0, 0, type_mask, q, 0, 0, 0, thrust
    )

    if duration > 0:
        start_time = time.time()
        while time.time() - start_time < duration:
            vehicle_obj.send_mavlink(msg)
            time.sleep(0.1)
        neutral_q = to_quaternion(0, 0, vehicle_obj.attitude.yaw)
        neutral_msg = vehicle_obj.message_factory.set_attitude_target_encode(
            0,0,0, 0b00000000, neutral_q, 0,0,0,0.5)
        vehicle_obj.send_mavlink(neutral_msg)
    else:
        vehicle_obj.send_mavlink(msg)
    print("Comando de atitude enviado.")

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Converte ângulos de Euler (roll, pitch, yaw) para uma lista Quaternion [w, x, y, z].
    """
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]

# --- Controle de Altitude ---
def set_altitude(vehicle_obj, target_alt_relative):
    """
    Define a altitude relativa do veículo.
    """
    current_location = vehicle_obj.location.global_relative_frame
    if current_location.lat is None or current_location.lon is None:
        print("Erro: Não é possível definir altitude sem uma localização GPS válida.")
        return

    target_location = LocationGlobalRelative(current_location.lat, current_location.lon, float(target_alt_relative))
    print(f"Definindo altitude para: {target_alt_relative}m")
    vehicle_obj.simple_goto(target_location)

    while True:
        current_alt = vehicle_obj.location.global_relative_frame.alt
        if current_alt is None:
            print("Aguardando dados de altitude válidos...")
            time.sleep(1)
            continue
        print(f"  Altitude atual: {current_alt:.2f}m, Alvo: {target_alt_relative:.2f}m")
        if abs(current_alt - target_alt_relative) < 0.5:
            print("Altitude alvo alcançada.")
            break
        time.sleep(1)

# --- Programa Principal (Exemplo de Uso) ---
try:
    altitude_decolagem = 1.5 # metros
    arm_and_takeoff(altitude_decolagem)

    print("\nRealizando RTL (Return to Launch)...")
    vehicle.mode = VehicleMode("RTL")
    while vehicle.mode.name != "RTL":
        print(" Aguardando modo RTL...")
        time.sleep(1)
    print("Veículo em modo RTL.")

    # Monitora o RTL até desarmar ou chegar perto do solo
    while vehicle.armed:
        alt_rtl = vehicle.location.global_relative_frame.alt
        print(f" Altitude RTL: {alt_rtl:.2f}m | Localização: {vehicle.location.global_relative_frame}")
        if alt_rtl is not None and alt_rtl < 0.5: # Considera próximo ao solo
            print("Próximo ao solo durante RTL, aguardando desarme...")
            # Pode levar alguns segundos para desarmar após tocar o solo
            time_near_ground = time.time()
            while vehicle.armed and (time.time() - time_near_ground < 10): # Espera até 10s pelo desarme
                time.sleep(0.5)
            break # Sai do loop principal de monitoramento RTL
        time.sleep(2)
    print("Veículo desarmado ou muito próximo do solo após RTL.")

except KeyboardInterrupt:
    print("\nInterrupção por teclado detectada. Tentando RTL...")
    if vehicle is not None and vehicle.mode.name != "RTL": # Só muda para RTL se não já estiver
        try:
            if vehicle.armed:
                print("Mudando para modo RTL...")
                vehicle.mode = VehicleMode("RTL")
                print("Modo RTL ativado. Monitore o pouso manual ou via GCS.")
            else:
                print("Veículo não estava armado.")
        except Exception as e_rtl_kb:
            print(f"Erro ao tentar ativar RTL na interrupção: {e_rtl_kb}")
    elif vehicle is not None and vehicle.mode.name == "RTL":
        print("Veículo já em modo RTL.")
    else:
        print("Veículo não conectado ou não armado.")

except Exception as e_main:
    print(f"ERRO DURANTE A EXECUÇÃO DO SCRIPT: {e_main}")
    import traceback
    traceback.print_exc()
    if vehicle is not None and vehicle.armed and vehicle.mode.name != "RTL":
        print("Tentando RTL de emergência devido ao erro...")
        try:
            vehicle.mode = VehicleMode("RTL")
            print("Modo RTL de emergência ativado. Monitore o pouso.")
        except Exception as e_rtl_emergency:
            print(f"Erro ao tentar ativar RTL de emergência: {e_rtl_emergency}")

finally:
    if vehicle is not None:
        print("Iniciando procedimento de finalização...")
        # Esperar um pouco mais se estiver em RTL para dar tempo de desarmar
        if vehicle.mode.name == "RTL" and vehicle.armed:
            print("Aguardando desarme final após RTL (até 15s)...")
            for _ in range(15):
                if not vehicle.armed:
                    print("Veículo desarmado.")
                    break
                time.sleep(1)
            else: # Se o loop terminar sem break
                if vehicle.armed:
                    print("Veículo ainda armado após espera RTL.")

        if vehicle.armed:
            print("Tentando desarmar o veículo explicitamente (se ainda armado)...")
            try:
                vehicle.armed = False
                time.sleep(1) # Dar tempo para o comando processar
                if vehicle.armed:
                    print("Falha ao desarmar o veículo explicitamente.")
            except Exception as e_disarm_final:
                print(f"Erro ao tentar desarmar na finalização: {e_disarm_final}")

        print("Fechando a conexão com o veículo.")
        vehicle.close()
    print("Script finalizado.")