from machine import Pin, ADC, I2C, reset
import ssd1306
import dht
import time

# Define la clase PID Adaptativo
class AdaptivePID:
    def __init__(self, kp, ki, kd, setpoint, large_error_threshold, small_error_threshold, min_kp, max_kp, min_ki, max_ki, min_kd, max_kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.large_error_threshold = large_error_threshold
        self.small_error_threshold = small_error_threshold
        self.min_kp = min_kp
        self.max_kp = max_kp
        self.min_ki = min_ki
        self.max_ki = max_ki
        self.min_kd = min_kd
        self.max_kd = max_kd
        self.prev_error = 0
        self.integral = 0
        self.prev_derivative = 0
        
    def compute(self, input_val):
        error = self.setpoint - input_val
        self.integral += error
        derivative = error - self.prev_error
        derivative_change = derivative - self.prev_derivative
        
        # Calculate output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Heuristic tuning logic (this is where the "intelligence" comes in)
        if abs(error) > self.large_error_threshold:
            # If error is large, increase kp to be more aggressive
            self.kp *= 1.1
        elif abs(error) < self.small_error_threshold:
            # If error is small, decrease kp to avoid overshooting
            self.kp *= 0.9
        
        # Update ki and kd based on the trend of the error
        if error * self.prev_error > 0:  # Error is consistent direction
            self.ki *= 1.05
        else:  # Error has changed direction
            self.ki *= 0.95
        #update kd
        if derivative_change > 0:  # Si la derivada del error está aumentando
            self.kd *= 1.1
        else:  # Si la derivada del error está disminuyendo o es cero
            self.kd *= 0.9
        # Ensure parameters stay within reasonable bounds
        self.kp = max(self.min_kp, min(self.max_kp, self.kp))
        self.ki = max(self.min_ki, min(self.max_ki, self.ki))
        self.kd = max(self.min_kd, min(self.max_kd, self.kd))

        self.prev_error = error
        self.prev_derivative = derivative  # Actualizar la derivada previa
        return output

sensor = dht.DHT22(Pin(5))
moisture_sensor = ADC(Pin(35))
relay_bulb = Pin(12, Pin.OUT)
relay_pump = Pin(27, Pin.OUT)
relay_fan = Pin(13, Pin.OUT)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

# Parámetros y controladores PID
setpoint_temp = 28
pid_temp = AdaptivePID(
    kp=2.303, ki=0.0085, kd=155.84, 
    setpoint=setpoint_temp, 
    large_error_threshold=5, small_error_threshold=1.4, 
    min_kp=1.15, max_kp=4.60, 
    min_ki=0.00425, max_ki=0.017,  #max-ki=5
    min_kd=77.92, max_kd=311.68
)
#aumentar small_error Si el sistema oscila o sobrepasa demasiado el setpoint
#Si el sistema reacciona demasiado lentamento,disminuir el large_error_threshold para que el controlador aumente su agresividad 
setpoint_hum = 30
pid_hum = AdaptivePID(
    kp=10, ki=0.0090, kd=100, 
    setpoint=setpoint_hum, 
    large_error_threshold=5, small_error_threshold=1.4, 
    min_kp=5, max_kp=15, 
    min_ki=0.0045, max_ki=0.0135, 
    min_kd=50, max_kd=150
)

# Filtro de media móvil
class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []

    def update(self, value):
        if len(self.values) >= self.window_size:
            self.values.pop(0)
        self.values.append(value)
        return sum(self.values) / len(self.values)
# Inicializar filtro con un tamaño de ventana de 3
moisture_filter = MovingAverageFilter(3)

def mostrar_datos_oled(temp, hum, moisture_pct):
    oled.fill(0)
    oled.text("PID adaptativo", 0, 0)
    oled.text("Setp Tem: {} C".format(setpoint_temp), 0, 10)
    oled.text("Setp Hum: {} %".format(setpoint_hum), 0, 20)
    oled.text("Temp: {:.2f} C".format(temp), 0, 30)
    oled.text("Hum: {:.2f} %".format(hum), 0, 40)
    oled.text("Humedad: {:.2f}%".format(moisture_filtered), 0, 50)
    oled.show()

ADC_MAX = 4095 #4095
ADC_MIN = 1130 #2251

def leer_moisture_sensor():
    valor = moisture_sensor.read()
    pct_raw = 100 - ((valor - ADC_MIN) / (ADC_MAX - ADC_MIN)) * 100  # Invertimos la lógica aquí
    pct_filtered = moisture_filter.update(pct_raw)  # Aplicamos el filtro de media móvil
    return pct_raw, pct_filtered  # Retornamos ambos valores: crudo y filtrado


MAX_RETRIES = 2
# Inicializamos las variables para almacenar las últimas lecturas válidas
last_valid_temp = None
last_valid_hum = None

while True:
    for _ in range(MAX_RETRIES):
        try:
            sensor.measure()
            temp = sensor.temperature()
            hum = sensor.humidity()
            last_valid_temp = temp  # Almacenamos la lectura válida
            last_valid_hum = hum
            break
        except Exception as e:
            print(f"Error al leer el sensor: {e}. Intentando de nuevo...")
            time.sleep(3)
    else:
        print("Fallo al leer el sensor después de varios intentos. Continuando con el último valor conocido.")
        if last_valid_temp is not None and last_valid_hum is not None:
            temp = last_valid_temp
            hum = last_valid_hum
        else:
            print("No hay valores válidos previos. Reiniciando el sistema.")
            reset()  # Reiniciar el microcontrolador
            continue

    moisture_raw, moisture_filtered = leer_moisture_sensor()  # Obtenemos ambos valores
    adc_value = moisture_sensor.read()
    output_temp = pid_temp.compute(temp)
    adc_value = moisture_sensor.read()
    output_hum = pid_hum.compute(moisture_filtered)  # usamos el valor filtrado para el PID
    # Imprimir los valores actuales del PID
    print(f"PID Temp - Kp: {pid_temp.kp}, Ki: {pid_temp.ki}, Kd: {pid_temp.kd}")
    print(f"PID Hum - Kp: {pid_hum.kp}, Ki: {pid_hum.ki}, Kd: {pid_hum.kd}")
    print("Sensor humedad Moisture Raw: {:.2f}%, Moisture Filtered: {:.2f}%".format(moisture_raw, moisture_filtered))


    if output_temp > 0:
        relay_bulb.value(1)
        relay_fan.value(0)
        print("Encendemos el bombillo")
    else:
        relay_bulb.value(0)
        relay_fan.value(1)
        print("Apagamos el bombillo y encendemos disipador")

    if output_hum > 0 and moisture_filtered < setpoint_hum:
        relay_pump.value(1)
        print("Encendemos la micro-bomba")
    else:
        relay_pump.value(0)
        print("Apagamos la micro-bomba")

    mostrar_datos_oled(temp, hum, moisture_filtered)  # Usamos el valor filtrado para mostrar en el OLED
    print("Temperature: {:.2f} C, HumedadR: {:.2f} %, HumedadS: {:.2f}%, ADC: {},valor_PID_humedad: {:.2f},valor_PID_temp: {:.2f} ".format(temp, hum, moisture_filtered, adc_value,output_hum,output_temp))
    time.sleep(10)
