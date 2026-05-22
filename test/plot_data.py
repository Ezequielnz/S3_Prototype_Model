import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def load_and_clean_data(filepath):
    """
    Lee un archivo CSV, limpia los errores de I2C/Wire y devuelve los datos limpios.
    """
    clean_data = []
    
    with open(filepath, 'r') as file:
        for line in file:
            line = line.strip()
            # Ignorar las líneas vacías o que contengan errores (ej: "[ 18956][E][Wire.cpp...")
            if not line or line.startswith('['):
                continue
            
            try:
                # Convertir los valores separados por comas a floats
                values = [float(x) for x in line.split(',')]
                clean_data.append(values)
            except ValueError:
                # Si hay algún otro error de conversión, ignorar la línea
                print(f"Advertencia: No se pudo procesar la línea: {line}")
                
    return np.array(clean_data)

def plot_acceleration(filepath, fs=100.0):
    """
    Limpia los datos del archivo y grafica la aceleración vs tiempo.
    fs: Frecuencia de muestreo estimada en Hz (muestras por segundo)
    """
    print(f"Procesando: {filepath}")
    
    # 1. Limpiar datos
    data = load_and_clean_data(filepath)
    
    if len(data) == 0:
        print("No se encontraron datos válidos.")
        return
        
    num_samples = data.shape[0]
    num_columns = data.shape[1]
    
    # 2. Crear vector de tiempo
    # Si no conocemos el tiempo exacto entre muestras, asumimos una frecuencia de muestreo (fs)
    time = np.arange(num_samples) / fs
    
    # 3. Graficar
    plt.figure(figsize=(10, 6))
    
    if num_columns == 3:
        # Probablemente muestra_01.csv (X, Y, Z)
        plt.plot(time, data[:, 0], label='Eje X', color='r', alpha=0.8)
        plt.plot(time, data[:, 1], label='Eje Y', color='g', alpha=0.8)
        plt.plot(time, data[:, 2], label='Eje Z', color='b', alpha=0.8)
        plt.title(f"Aceleración Traslacional (3 Ejes) - {os.path.basename(filepath)}")
        plt.ylabel("Aceleración (g o m/s²)")
    elif num_columns == 1:
        # Probablemente muestra_02.csv (1 Eje o Magnitud)
        plt.plot(time, data[:, 0], label='Aceleración', color='b', alpha=0.8)
        plt.title(f"Aceleración Traslacional (1 Eje) - {os.path.basename(filepath)}")
        plt.ylabel("Aceleración (g o m/s²)")
    else:
        # Caso genérico
        for i in range(num_columns):
            plt.plot(time, data[:, i], label=f'Columna {i+1}', alpha=0.8)
        plt.title(f"Aceleración - {os.path.basename(filepath)}")
        plt.ylabel("Valor")

    plt.xlabel("Tiempo (segundos) - *Asumiendo fs=100Hz")
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Rutas de los archivos
    file1 = r"c:\Users\Usuario\Documents\Workspace\AIoT\S3_Prototype_Model\test\muestra_01.csv"
    file2 = r"c:\Users\Usuario\Documents\Workspace\AIoT\S3_Prototype_Model\test\muestra_02.csv"
    
    # Graficar el primer archivo
    if os.path.exists(file1):
        plot_acceleration(file1, fs=100.0) # Ajusta fs según la frecuencia real de tu ESP32
    else:
        print(f"No se encontró el archivo: {file1}")
        
    # Graficar el segundo archivo
    if os.path.exists(file2):
        plot_acceleration(file2, fs=100.0) # Ajusta fs según la frecuencia real de tu ESP32
    else:
        print(f"No se encontró el archivo: {file2}")
